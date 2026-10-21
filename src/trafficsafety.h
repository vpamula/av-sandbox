#pragma once

#include "vehicle.h"

#include <cstddef>
#include <vector>

class TrafficSafety {
public:
    // Vehicle zero is the ego car; manual input is applied to it when requested.
    void step(
        const std::vector<Vehicle*>& vehicles,
        bool manualEgoControl,
        float dt,
        std::size_t priorityParticipantStart
    ) const;
};
