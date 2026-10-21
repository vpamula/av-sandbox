#include "lane.h"

#include "raymath.h"

#include <algorithm>
#include <utility>

Lane::Lane(std::vector<Vector2> points) : waypoints(std::move(points)) {}

Vector2 Lane::getDirectionAt(std::size_t waypointIndex) const {
    if (waypoints.size() < 2) {
        return { 1.0f, 0.0f };
    }

    const std::size_t current = std::min(waypointIndex, waypoints.size() - 1);
    const std::size_t previous = current == 0 ? current : current - 1;
    const std::size_t next = current + 1 < waypoints.size() ? current + 1 : current;
    Vector2 direction = Vector2Subtract(waypoints[next], waypoints[previous]);
    const float length = Vector2Length(direction);

    if (length <= 0.0001f) {
        return { 1.0f, 0.0f };
    }

    return Vector2Scale(direction, 1.0f / length);
}

const std::vector<Vector2>& Lane::getWaypoints() const {
    return waypoints;
}
