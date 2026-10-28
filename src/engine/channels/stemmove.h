#pragma once

// Stem "moves" (Asad's build): one-shot performance effects for a single stem,
// in the spirit of the stem FX on other DJ software. Each move plays out over a
// couple of beats and leaves the stem silent; the deck then mutes the stem.
//
//   EchoOut      the stem keeps playing for a beat into an echo, then the live
//                stem cuts out and only the echoes ring on and fade away
//   VinylBrake   the stem slows to a stop like a record with the motor switched
//                off, while the other stems keep playing
//   ReverbOut    like EchoOut but into a big reverb
//   FilterSweep  a high-pass filter sweeps up until the stem has gone
//
// Runs on the audio thread: no allocation after construction, no locks.

#include <array>
#include <cstddef>
#include <vector>

namespace mixxx {

class StemMove {
  public:
    enum class Type : int {
        None = 0,
        EchoOut = 1,
        VinylBrake = 2,
        ReverbOut = 3,
        FilterSweep = 4,
    };
    static constexpr int kTypeCount = 5;

    // Buffers are sized for sample rates up to this; higher rates still work
    // with proportionally shorter maximum delay/brake times.
    static constexpr double kMaxSampleRate = 96000.0;

    StemMove();

    /// Starts a move. bpm <= 0 means unknown (120 is assumed).
    void start(Type type, double sampleRate, double bpm);
    /// Stops immediately and forgets everything.
    void reset();

    Type type() const {
        return m_type;
    }
    /// True while the move is audible (not yet finished).
    bool isPlaying() const {
        return m_type != Type::None && !m_finished;
    }
    /// True once the move has faded to silence; the stem should now be muted.
    bool isFinished() const {
        return m_type != Type::None && m_finished;
    }

    /// Processes an interleaved stereo buffer in place.
    void process(float* pInOut, std::size_t numFrames);

  private:
    void processEcho(float* pInOut, std::size_t numFrames);
    void processReverb(float* pInOut, std::size_t numFrames);
    void processVinyl(float* pInOut, std::size_t numFrames);
    void processFilter(float* pInOut, std::size_t numFrames);
    void updateTail(const float* pBuf, std::size_t numFrames);
    float reverbTick(int ch, float in);
    void setHighPass(double cutoffHz);

    Type m_type;
    bool m_finished;
    double m_sampleRate;
    double m_framesPerBeat;
    std::size_t m_pos;       // frames since start
    std::size_t m_feedFrames; // frames the live stem keeps playing / feeding
    std::size_t m_maxFrames;  // hard limit for the whole move
    std::size_t m_quietFrames;

    // shared stereo ring buffer: echo delay line / vinyl capture
    std::vector<float> m_ring;
    std::size_t m_ringFrames;
    std::size_t m_writePos;
    std::size_t m_echoDelayFrames;
    double m_readPos; // vinyl: fractional read position (frames, absolute count)
    double m_rate;
    std::size_t m_brakeFrames;

    // reverb (Freeverb-style): 8 combs + 4 allpasses per channel
    static constexpr int kCombs = 8;
    static constexpr int kAllpasses = 4;
    struct Comb {
        std::vector<float> buf;
        std::size_t len = 1;
        std::size_t idx = 0;
        float store = 0.0f;
    };
    struct Allpass {
        std::vector<float> buf;
        std::size_t len = 1;
        std::size_t idx = 0;
    };
    std::array<std::array<Comb, kCombs>, 2> m_combs;
    std::array<std::array<Allpass, kAllpasses>, 2> m_allpasses;

    // high-pass biquad (RBJ), direct form I, per channel
    double m_b0, m_b1, m_b2, m_a1, m_a2;
    std::array<std::array<double, 4>, 2> m_hpState; // x1, x2, y1, y2
    std::size_t m_sweepFrames;
};

} // namespace mixxx
