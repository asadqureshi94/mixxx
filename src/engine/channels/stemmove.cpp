#include "engine/channels/stemmove.h"

#include <algorithm>
#include <cmath>

namespace mixxx {

namespace {

// How each move is shaped (times in beats unless noted)
constexpr double kFeedBeats = 1.0;         // echo/reverb: live stem keeps playing this long
constexpr double kEchoDelayBeats = 0.5;    // echo spacing
constexpr float kEchoFeedback = 0.6f;      // each echo is 60% of the one before
constexpr float kEchoWet = 0.6f;
constexpr double kEchoMaxTailBeats = 16.0; // safety limit
constexpr float kReverbFeedback = 0.95f;   // long tail
constexpr float kReverbDamping = 0.25f;
constexpr float kReverbWet = 4.0f;          // swells in over the first beat
constexpr double kReverbMaxTailSeconds = 8.0;
constexpr double kBrakeBeats = 2.0;        // vinyl brake length...
constexpr double kBrakeMinSeconds = 0.8;   // ...kept within these limits
constexpr double kBrakeMaxSeconds = 2.0;
constexpr double kSweepSeconds = 1.5;      // filter sweep length
constexpr double kSweepStartHz = 30.0;
constexpr double kSweepEndHz = 12000.0;
constexpr double kSweepQ = 1.2;
constexpr double kDryFadeSeconds = 0.02;   // fade when the live stem cuts out
constexpr float kQuietLevel = 3e-4f;       // about -70 dB
constexpr double kQuietSeconds = 0.25;
constexpr double kRingSeconds = 2.0;

// Freeverb tunings at 44.1 kHz
constexpr std::array<int, 8> kCombTuning = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
constexpr std::array<int, 4> kAllpassTuning = {556, 441, 341, 225};
constexpr int kStereoSpread = 23;
constexpr float kReverbInputGain = 0.015f;

std::size_t framesFor(double seconds, double sampleRate) {
    return static_cast<std::size_t>(std::max(0.0, seconds * sampleRate));
}

} // namespace

StemMove::StemMove()
        : m_type(Type::None),
          m_finished(false),
          m_sampleRate(44100.0),
          m_framesPerBeat(22050.0),
          m_pos(0),
          m_feedFrames(0),
          m_maxFrames(0),
          m_quietFrames(0),
          m_ringFrames(1),
          m_writePos(0),
          m_echoDelayFrames(1),
          m_readPos(0.0),
          m_rate(1.0),
          m_brakeFrames(1),
          m_b0(1.0),
          m_b1(0.0),
          m_b2(0.0),
          m_a1(0.0),
          m_a2(0.0),
          m_hpState{},
          m_sweepFrames(1) {
    m_ring.assign(2 * framesFor(kRingSeconds, kMaxSampleRate), 0.0f);
    const double scale = kMaxSampleRate / 44100.0;
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < kCombs; ++i) {
            m_combs[ch][i].buf.assign(
                    static_cast<std::size_t>((kCombTuning[i] + kStereoSpread) * scale) + 2,
                    0.0f);
        }
        for (int i = 0; i < kAllpasses; ++i) {
            m_allpasses[ch][i].buf.assign(
                    static_cast<std::size_t>((kAllpassTuning[i] + kStereoSpread) * scale) + 2,
                    0.0f);
        }
    }
}

void StemMove::reset() {
    m_type = Type::None;
    m_finished = false;
    m_pos = 0;
}

void StemMove::start(Type type, double sampleRate, double bpm) {
    reset();
    if (type == Type::None || sampleRate <= 0.0) {
        return;
    }
    m_type = type;
    m_sampleRate = sampleRate;
    if (!(bpm > 20.0 && bpm < 400.0)) {
        bpm = 120.0;
    }
    m_framesPerBeat = sampleRate * 60.0 / bpm;
    m_quietFrames = 0;
    m_writePos = 0;
    m_ringFrames = std::max<std::size_t>(1,
            std::min(m_ring.size() / 2, framesFor(kRingSeconds, sampleRate)));
    std::fill(m_ring.begin(), m_ring.end(), 0.0f);

    switch (type) {
    case Type::EchoOut: {
        m_feedFrames = static_cast<std::size_t>(kFeedBeats * m_framesPerBeat);
        m_echoDelayFrames = std::clamp<std::size_t>(
                static_cast<std::size_t>(kEchoDelayBeats * m_framesPerBeat),
                1,
                m_ringFrames - 1);
        m_maxFrames = m_feedFrames +
                static_cast<std::size_t>(kEchoMaxTailBeats * m_framesPerBeat);
        break;
    }
    case Type::ReverbOut: {
        m_feedFrames = static_cast<std::size_t>(kFeedBeats * m_framesPerBeat);
        m_maxFrames = m_feedFrames + framesFor(kReverbMaxTailSeconds, sampleRate);
        const double scale = sampleRate / 44100.0;
        for (int ch = 0; ch < 2; ++ch) {
            const int spread = ch == 0 ? 0 : kStereoSpread;
            for (int i = 0; i < kCombs; ++i) {
                Comb& c = m_combs[ch][i];
                c.len = std::clamp<std::size_t>(
                        static_cast<std::size_t>((kCombTuning[i] + spread) * scale),
                        1,
                        c.buf.size());
                c.idx = 0;
                c.store = 0.0f;
                std::fill(c.buf.begin(), c.buf.end(), 0.0f);
            }
            for (int i = 0; i < kAllpasses; ++i) {
                Allpass& a = m_allpasses[ch][i];
                a.len = std::clamp<std::size_t>(
                        static_cast<std::size_t>((kAllpassTuning[i] + spread) * scale),
                        1,
                        a.buf.size());
                a.idx = 0;
                std::fill(a.buf.begin(), a.buf.end(), 0.0f);
            }
        }
        break;
    }
    case Type::VinylBrake: {
        const double seconds = std::clamp(kBrakeBeats * m_framesPerBeat / sampleRate,
                kBrakeMinSeconds,
                kBrakeMaxSeconds);
        // the read position falls behind by up to half the brake time
        m_brakeFrames = std::min(framesFor(seconds, sampleRate), 2 * (m_ringFrames - 2));
        m_brakeFrames = std::max<std::size_t>(m_brakeFrames, 1);
        m_readPos = 0.0;
        m_rate = 1.0;
        m_feedFrames = m_brakeFrames;
        m_maxFrames = m_brakeFrames + 1;
        break;
    }
    case Type::FilterSweep: {
        m_sweepFrames = std::max<std::size_t>(1, framesFor(kSweepSeconds, sampleRate));
        m_feedFrames = m_sweepFrames;
        m_maxFrames = m_sweepFrames + framesFor(kDryFadeSeconds, sampleRate) + 1;
        for (auto& st : m_hpState) {
            st.fill(0.0);
        }
        setHighPass(kSweepStartHz);
        break;
    }
    case Type::None:
        break;
    }
}

void StemMove::process(float* pInOut, std::size_t numFrames) {
    if (m_type == Type::None) {
        return;
    }
    if (m_finished) {
        std::fill(pInOut, pInOut + 2 * numFrames, 0.0f);
        return;
    }
    switch (m_type) {
    case Type::EchoOut:
        processEcho(pInOut, numFrames);
        break;
    case Type::ReverbOut:
        processReverb(pInOut, numFrames);
        break;
    case Type::VinylBrake:
        processVinyl(pInOut, numFrames);
        break;
    case Type::FilterSweep:
        processFilter(pInOut, numFrames);
        break;
    case Type::None:
        break;
    }
}

// Gain of the live (dry) stem at frame m_pos: 1 while feeding, then a short fade to 0
static float dryGainAt(std::size_t pos, std::size_t feedFrames, std::size_t fadeFrames) {
    if (pos < feedFrames) {
        return 1.0f;
    }
    if (pos >= feedFrames + fadeFrames) {
        return 0.0f;
    }
    return 1.0f - static_cast<float>(pos - feedFrames) / static_cast<float>(fadeFrames);
}

void StemMove::processEcho(float* pInOut, std::size_t numFrames) {
    const std::size_t fade = std::max<std::size_t>(1, framesFor(kDryFadeSeconds, m_sampleRate));
    for (std::size_t i = 0; i < numFrames; ++i) {
        const float dry = dryGainAt(m_pos, m_feedFrames, fade);
        const std::size_t readPos = (m_writePos + m_ringFrames - m_echoDelayFrames) % m_ringFrames;
        for (int ch = 0; ch < 2; ++ch) {
            const float in = pInOut[2 * i + ch];
            const float echo = m_ring[2 * readPos + ch];
            m_ring[2 * m_writePos + ch] = in * dry + echo * kEchoFeedback;
            pInOut[2 * i + ch] = in * dry + echo * kEchoWet;
        }
        m_writePos = (m_writePos + 1) % m_ringFrames;
        ++m_pos;
    }
    updateTail(pInOut, numFrames);
}

float StemMove::reverbTick(int ch, float in) {
    float out = 0.0f;
    const float input = in * kReverbInputGain;
    for (Comb& c : m_combs[ch]) {
        const float y = c.buf[c.idx];
        c.store = y * (1.0f - kReverbDamping) + c.store * kReverbDamping;
        c.buf[c.idx] = input + c.store * kReverbFeedback;
        if (++c.idx >= c.len) {
            c.idx = 0;
        }
        out += y;
    }
    for (Allpass& a : m_allpasses[ch]) {
        const float buffered = a.buf[a.idx];
        const float y = -out + buffered;
        a.buf[a.idx] = out + buffered * 0.5f;
        if (++a.idx >= a.len) {
            a.idx = 0;
        }
        out = y;
    }
    return out;
}

void StemMove::processReverb(float* pInOut, std::size_t numFrames) {
    const std::size_t fade = std::max<std::size_t>(1, framesFor(kDryFadeSeconds, m_sampleRate));
    for (std::size_t i = 0; i < numFrames; ++i) {
        const float dry = dryGainAt(m_pos, m_feedFrames, fade);
        const float inL = pInOut[2 * i] * dry;
        const float inR = pInOut[2 * i + 1] * dry;
        const float mono = 0.5f * (inL + inR);
        const float wetL = reverbTick(0, mono);
        const float wetR = reverbTick(1, mono);
        // the reverb swells in while the stem is still playing, then rings on alone
        const float swell = m_pos < m_feedFrames
                ? static_cast<float>(m_pos) / static_cast<float>(m_feedFrames)
                : 1.0f;
        const float duck = 1.0f - 0.3f * swell; // make room for the reverb
        pInOut[2 * i] = inL * duck + wetL * kReverbWet * swell;
        pInOut[2 * i + 1] = inR * duck + wetR * kReverbWet * swell;
        ++m_pos;
    }
    updateTail(pInOut, numFrames);
}

void StemMove::processVinyl(float* pInOut, std::size_t numFrames) {
    for (std::size_t i = 0; i < numFrames; ++i) {
        // capture the live stem
        const std::size_t w = m_pos % m_ringFrames;
        m_ring[2 * w] = pInOut[2 * i];
        m_ring[2 * w + 1] = pInOut[2 * i + 1];
        // play it back at a falling speed, like a record slowing down
        const double t = static_cast<double>(m_pos) / static_cast<double>(m_brakeFrames);
        m_rate = std::max(0.0, 1.0 - t);
        float outL = 0.0f;
        float outR = 0.0f;
        if (m_rate > 0.0) {
            const auto i0 = static_cast<std::size_t>(m_readPos);
            const float frac = static_cast<float>(m_readPos - static_cast<double>(i0));
            const std::size_t a = i0 % m_ringFrames;
            const std::size_t b = std::min<std::size_t>(i0 + 1, m_pos) % m_ringFrames;
            // soften the very end so it doesn't click
            const float gain = t > 0.85 ? static_cast<float>((1.0 - t) / 0.15) : 1.0f;
            outL = (m_ring[2 * a] * (1.0f - frac) + m_ring[2 * b] * frac) * gain;
            outR = (m_ring[2 * a + 1] * (1.0f - frac) + m_ring[2 * b + 1] * frac) * gain;
            m_readPos += m_rate;
        }
        pInOut[2 * i] = outL;
        pInOut[2 * i + 1] = outR;
        ++m_pos;
    }
    if (m_pos >= m_brakeFrames) {
        m_finished = true;
    }
}

void StemMove::setHighPass(double cutoffHz) {
    const double f = std::clamp(cutoffHz, 10.0, 0.45 * m_sampleRate);
    const double w0 = 2.0 * 3.14159265358979323846 * f / m_sampleRate;
    const double alpha = std::sin(w0) / (2.0 * kSweepQ);
    const double cosw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    m_b0 = (1.0 + cosw) / 2.0 / a0;
    m_b1 = -(1.0 + cosw) / a0;
    m_b2 = (1.0 + cosw) / 2.0 / a0;
    m_a1 = -2.0 * cosw / a0;
    m_a2 = (1.0 - alpha) / a0;
}

void StemMove::processFilter(float* pInOut, std::size_t numFrames) {
    constexpr std::size_t kBlock = 32; // update the cutoff every 32 frames
    const std::size_t fade = std::max<std::size_t>(1, framesFor(kDryFadeSeconds, m_sampleRate));
    for (std::size_t i = 0; i < numFrames; ++i) {
        if (m_pos % kBlock == 0) {
            const double t = std::min(1.0,
                    static_cast<double>(m_pos) / static_cast<double>(m_sweepFrames));
            setHighPass(kSweepStartHz * std::pow(kSweepEndHz / kSweepStartHz, t));
        }
        const float gain = dryGainAt(m_pos, m_sweepFrames, fade);
        for (int ch = 0; ch < 2; ++ch) {
            auto& s = m_hpState[ch];
            const double x = pInOut[2 * i + ch];
            const double y = m_b0 * x + m_b1 * s[0] + m_b2 * s[1] - m_a1 * s[2] - m_a2 * s[3];
            s[1] = s[0];
            s[0] = x;
            s[3] = s[2];
            s[2] = y;
            pInOut[2 * i + ch] = static_cast<float>(y) * gain;
        }
        ++m_pos;
    }
    if (m_pos >= m_sweepFrames + fade) {
        m_finished = true;
    }
}

void StemMove::updateTail(const float* pBuf, std::size_t numFrames) {
    if (m_pos <= m_feedFrames) {
        return;
    }
    float peak = 0.0f;
    for (std::size_t i = 0; i < 2 * numFrames; ++i) {
        peak = std::max(peak, std::fabs(pBuf[i]));
    }
    if (peak < kQuietLevel) {
        m_quietFrames += numFrames;
    } else {
        m_quietFrames = 0;
    }
    if (m_quietFrames >= framesFor(kQuietSeconds, m_sampleRate) || m_pos >= m_maxFrames) {
        m_finished = true;
    }
}

} // namespace mixxx
