#pragma once

#include <cstdlib>
#include <limits>

#include "typedefs.h"

static thread_local unsigned int rng_state = 8917264;

[[maybe_unused]] static unsigned int rand_pcg() {
    const unsigned int state = rng_state;
    rng_state = rng_state * 747796405u + 2891336453u;
    const unsigned int word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

static unsigned int XOrShift32() {
    unsigned int x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static Real rand_real() {
    return  static_cast<Real>(XOrShift32()) / (static_cast<Real>(std::numeric_limits<unsigned int>::max()) + 1);
}