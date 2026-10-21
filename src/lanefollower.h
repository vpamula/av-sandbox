#pragma once

#include "vehicle.h"
#include "raylib.h"

#include <cstddef>
#include <vector>

class LaneFollower {
public:
    LaneFollower();

    void update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt);
    void reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints);

private:
    std::size_t currentWaypointIndex = 0;
    float steeringGain = 5.0f;
    std::size_t lookaheadPoints = 5;
};
