#pragma once

#include "lanefollower.h"
#include "vehicle.h"
#include "world.h"

#include <cstddef>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

struct StopSignParticipant {
    Vehicle* vehicle = nullptr;
    LaneFollower* controller = nullptr;
};

class StopSignCoordinator {
public:
    void update(const World& world, const std::vector<StopSignParticipant>& participants);

private:
    struct JunctionQueue {
        std::deque<StopSignParticipant> waiting;
        std::optional<StopSignParticipant> active;
    };

    std::unordered_map<std::size_t, JunctionQueue> junctions;
};
