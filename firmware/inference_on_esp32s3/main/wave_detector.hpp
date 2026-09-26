#pragma once

// --------------------------------------------------------------------------
// WaveDetector -- turns a stream of per-frame hand detections into a
// "the hand is waving" decision. Pure C++ with no ESP-IDF / esp-dl
// dependency, so it is unit-tested on the host (test/test_wave_detector.cpp).
//
// Why this exists: esp-dl's hand_detect model answers "where is a hand in THIS
// frame" -- a waving hand is a property of motion over TIME, which a
// single-frame model cannot see. So we track the hand box's horizontal centre
// over a sliding window and count *direction reversals*: a wave is the hand
// going right, then left, then right, ... with a swing bigger than jitter.
//
//   x(t) over the window, e.g.:  right, left, right, left  = 3 turning
//   points (reversals), each swing >= thr  =>  WAVING
//
// Swing threshold is a fraction of the hand's own box width, so the decision is
// independent of how far the hand is from the camera (a far hand is small and
// waves through fewer pixels).
// --------------------------------------------------------------------------

#include <cstdint>
#include <cstddef>
#include <cmath>

struct WaveConfig {
    int64_t window_us     = 2000000; // history considered when counting reversals
    int     min_reversals = 3;       // turning points needed (3 ~= 1.5 back-and-forth cycles)
    float   amp_frac      = 0.30f;   // min swing, as a fraction of mean hand box width
    float   min_amp_px    = 6.0f;    // ...but never below this (box jitter floor)
    int64_t lost_us       = 600000;  // no hand for this long -> forget history
    int64_t hold_us       = 800000;  // keep reporting "waving" this long after last reversal
};

class WaveDetector {
public:
    explicit WaveDetector(const WaveConfig &cfg = WaveConfig()) : cfg_(cfg) {}

    // Feed one frame's result. `present` = a hand was detected; cx = box centre
    // x in pixels, width = box width in pixels (ignored when !present).
    // Returns true while the hand is considered to be waving.
    bool update(int64_t ts_us, bool present, float cx, float width) {
        if (!present) {
            if (n_ > 0 && ts_us - samples_[(head_ + n_ - 1) % kMax].ts > cfg_.lost_us) reset_history();
            return waving(ts_us);
        }

        // Drop samples that fell out of the window, then append.
        while (n_ > 0 && ts_us - samples_[head_].ts > cfg_.window_us) { head_ = (head_ + 1) % kMax; n_--; }
        if (n_ == kMax) { head_ = (head_ + 1) % kMax; n_--; }
        samples_[(head_ + n_) % kMax] = {ts_us, cx, width};
        n_++;

        last_reversals_ = count_reversals();
        if (last_reversals_ >= cfg_.min_reversals) last_wave_ts_ = ts_us;
        return waving(ts_us);
    }

    // Reversals counted on the most recent update (for logging/tuning).
    int reversals() const { return last_reversals_; }

    bool waving(int64_t now_us) const {
        return last_wave_ts_ != kNever && now_us - last_wave_ts_ <= cfg_.hold_us;
    }

    void reset() { reset_history(); last_wave_ts_ = kNever; }

private:
    static constexpr int     kMax   = 48;   // > window * max fps (2s * ~20fps)
    static constexpr int64_t kNever = INT64_MIN;

    struct Sample { int64_t ts; float cx; float w; };

    void reset_history() { head_ = 0; n_ = 0; last_reversals_ = 0; }

    // Hysteresis turning-point counter: a reversal only counts once the hand
    // has moved `thr` back from the last extreme, so small jitter is ignored.
    int count_reversals() const {
        if (n_ < 3) return 0;
        float mean_w = 0;
        for (int i = 0; i < n_; i++) mean_w += samples_[(head_ + i) % kMax].w;
        mean_w /= n_;
        const float thr = std::fmax(cfg_.min_amp_px, cfg_.amp_frac * mean_w);

        int   dir = 0, rev = 0;
        float ext = samples_[head_].cx;
        for (int i = 1; i < n_; i++) {
            const float x = samples_[(head_ + i) % kMax].cx;
            if (dir == 0) {
                if (x - ext >= thr)      { dir = +1; ext = x; }
                else if (ext - x >= thr) { dir = -1; ext = x; }
            } else if (dir > 0) {
                if (x > ext)             ext = x;
                else if (ext - x >= thr) { rev++; dir = -1; ext = x; }
            } else {
                if (x < ext)             ext = x;
                else if (x - ext >= thr) { rev++; dir = +1; ext = x; }
            }
        }
        return rev;
    }

    WaveConfig cfg_;
    Sample     samples_[kMax];
    int        head_ = 0, n_ = 0;
    int        last_reversals_ = 0;
    int64_t    last_wave_ts_ = kNever;
};
