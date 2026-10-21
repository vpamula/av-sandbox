#pragma once

#include "vehicle.h"

#include <cstddef>
#include <deque>
#include <unordered_map>
#include <vector>

class World;

class TrafficSafety {
public:
    // Vehicle zero is the ego car; manual input is applied to it when requested.
    void step(
        const World& world,
        const std::vector<Vehicle*>& vehicles,
        bool manualEgoControl,
        float dt,
        std::size_t priorityParticipantStart,
        std::size_t pedestrianPriorityStart
    );

private:
    struct JunctionRequest {
        Vehicle* vehicle = nullptr;
        std::size_t arrivalStep = 0;
    };

    struct JunctionReservation {
        Vehicle* owner = nullptr;
        bool ownerEntered = false;
        std::deque<JunctionRequest> waiting;
    };

    std::unordered_map<std::size_t, JunctionReservation> uncontrolledJunctions;
};
