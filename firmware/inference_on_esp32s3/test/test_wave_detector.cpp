// Host-side unit test for main/wave_detector.hpp (no ESP-IDF needed):
//   g++ -std=c++17 -Wall -I main test/test_wave_detector.cpp -o test/test_wave_detector
//   ./test/test_wave_detector
#include "wave_detector.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

static const double PI = 3.14159265358979;

// Drive `d` with a hand whose centre x follows fn(t_sec), sampled at `fps`.
// Sets *ever if the detector reported waving at any sample.
template <class F>
static void run(WaveDetector &d, double secs, double fps, F fn, bool *ever = nullptr, float w = 80.0f) {
    for (double t = 0; t < secs; t += 1.0 / fps) {
        bool r = d.update((int64_t)(t * 1e6), true, (float)fn(t), w);
        if (ever && r) *ever = true;
    }
}

int main() {
    { // 1.5 Hz wave, +-30px swing on an 80px hand, at the ~8 fps the S3 achieves
        WaveDetector d; bool ever = false;
        run(d, 3.0, 8.0, [](double t) { return 160 + 30 * std::sin(2 * PI * 1.5 * t); }, &ever);
        CHECK(ever);
    }
    { // slower 0.8 Hz wave at only 4 fps still detected
        WaveDetector d; bool ever = false;
        run(d, 4.0, 4.0, [](double t) { return 160 + 30 * std::sin(2 * PI * 0.8 * t); }, &ever);
        CHECK(ever);
    }
    { // stationary hand with +-2px detector jitter -> never waving
        WaveDetector d; bool ever = false; srand(1);
        run(d, 5.0, 8.0, [](double) { return 160.0 + (rand() % 5 - 2); }, &ever);
        CHECK(!ever);
    }
    { // single sweep across the frame (moving, not waving)
        WaveDetector d; bool ever = false;
        run(d, 2.0, 8.0, [](double t) { return 40 + 120 * t; }, &ever);
        CHECK(!ever);
    }
    { // small tremor (+-4px on an 80px hand) is under the 0.3*80=24px threshold
        WaveDetector d; bool ever = false;
        run(d, 4.0, 8.0, [](double t) { return 160 + 4 * std::sin(2 * PI * 2.0 * t); }, &ever);
        CHECK(!ever);
    }
    { // hold: still reports waving just after motion stops, then clears
        WaveDetector d;
        run(d, 3.0, 8.0, [](double t) { return 160 + 30 * std::sin(2 * PI * 1.5 * t); });
        CHECK(d.update(3100000, true, 160, 80));
        bool later = true;
        for (int64_t t = 3100000; t < 8000000; t += 125000) later = d.update(t, true, 160, 80);
        CHECK(!later);
    }
    { // hand gone longer than lost_us: history is forgotten
        WaveDetector d;
        run(d, 1.0, 8.0, [](double t) { return 160 + 30 * std::sin(2 * PI * 1.5 * t); });
        for (int64_t t = 1000000; t < 3000000; t += 125000) d.update(t, false, 0, 0);
        CHECK(!d.update(3000000, true, 160, 80));
        CHECK(d.reversals() == 0);
    }
    { // far-away hand (30px box) waving through proportionally fewer px is still detected
        WaveDetector d; bool ever = false;
        run(d, 3.0, 8.0, [](double t) { return 160 + 12 * std::sin(2 * PI * 1.5 * t); }, &ever, 30.0f);
        CHECK(ever);
    }

    printf(failures ? "%d FAILED\n" : "all wave_detector tests passed\n", failures);
    return failures != 0;
}
