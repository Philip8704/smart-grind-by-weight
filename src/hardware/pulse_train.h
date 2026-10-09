#pragma once
#include <stddef.h>
#include <stdint.h>

// Builds an RMT symbol sequence for `count` motor pulses of on_us HIGH followed by
// off_us LOW, for a single transmission with no hardware looping.
//
// Each RMT symbol carries two (level, duration) halves with a 15-bit duration, so at
// 1MHz no half can exceed 32767us. Longer periods are split across as many halves as
// they need, which keeps every period exact. (Looping one symbol to make a long pulse
// does not: the loop repeats the whole symbol, remainder included.)
//
// A template over the symbol type so it can be checked off-device against a plain
// struct with the same four fields as rmt_symbol_word_t.
//
// Returns the number of symbols written, or 0 if `capacity` is too small or the input
// is degenerate - callers must treat 0 as "do not transmit".
template <typename Symbol>
size_t build_pulse_train(Symbol* out, size_t capacity,
                         uint32_t on_us, uint32_t off_us, uint32_t count) {
    constexpr uint32_t kMaxHalfUs = 32767;
    if (!out || capacity == 0 || on_us == 0 || count == 0) {
        return 0;
    }

    size_t halves = 0;
    const size_t max_halves = capacity * 2;

    auto put = [&](uint32_t level, uint32_t us) -> bool {
        while (us > 0) {
            if (halves >= max_halves) {
                return false;
            }
            uint32_t chunk = (us > kMaxHalfUs) ? kMaxHalfUs : us;
            Symbol& symbol = out[halves / 2];
            if ((halves % 2) == 0) {
                symbol.level0 = level;
                symbol.duration0 = chunk;
            } else {
                symbol.level1 = level;
                symbol.duration1 = chunk;
            }
            halves++;
            us -= chunk;
        }
        return true;
    };

    for (uint32_t i = 0; i < count; i++) {
        if (!put(1, on_us)) return 0;
        if (!put(0, off_us)) return 0;
    }
    // A symbol half with duration 0 is an end-of-data marker to the RMT, so an odd
    // number of halves is padded with a 1us LOW rather than left empty. The driver
    // appends its own end marker after the last symbol.
    if (halves % 2) {
        if (!put(0, 1)) return 0;
    }
    return halves / 2;
}

// Total HIGH and overall duration of what build_pulse_train would emit, in us
inline uint64_t pulse_train_duration_us(uint32_t on_us, uint32_t off_us, uint32_t count) {
    return (uint64_t)(on_us + off_us) * count;
}
