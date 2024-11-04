#pragma once

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <limits>

typedef double Real;

static thread_local unsigned int rng_state = rand();

[[maybe_unused]] static inline unsigned int rand_pcg() {
    unsigned int state = rng_state;
    rng_state = rng_state * 747796405u + 2891336453u;
    unsigned int word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

static inline unsigned int XOrShift32() {
    unsigned int x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static inline Real rand_real() {
    auto a = (Real) XOrShift32() / ((Real) std::numeric_limits<unsigned int>::max() + 1);
    assert(0 <= a && a <= 1);
    return a;
}