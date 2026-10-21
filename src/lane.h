#pragma once

#include "raylib.h"

#include <cstddef>
#include <vector>

class Lane {
public:
    explicit Lane(std::vector<Vector2> waypoints);

    Vector2 getDirectionAt(std::size_t waypointIndex) const;
    const std::vector<Vector2>& getWaypoints() const;

private:
    std::vector<Vector2> waypoints;
};
