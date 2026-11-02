#ifndef WATER_ANIMATION_H
#define WATER_ANIMATION_H

#include <stdint.h>

namespace WaterAnimation {

    // Initialize the water animation.
    void begin();

    // Reset the water distribution to its initial state.
    void reset(uint32_t nowMs);

    // Update the water simulation.
    // Call frequently from general_run().
    void update(uint32_t nowMs);

}

#endif