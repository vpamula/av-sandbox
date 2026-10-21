#include "lane.h"

#include <cmath>

namespace {
constexpr float WAYPOINT_SPACING = 100.0f;
}

Lane::Lane(float centerX, float centerY, float heading, float length)
    : centerX(centerX), centerY(centerY), heading(heading), length(length) {}

std::vector<Vector2> Lane::generateWaypoints() const {
    std::vector<Vector2> waypoints;
    const Vector2 direction = getDirection();

    for (float distance = -length / 2.0f; distance <= length / 2.0f; distance += WAYPOINT_SPACING) {
        waypoints.push_back({
            centerX + direction.x * distance,
            centerY + direction.y * distance
        });
    }

    return waypoints;
}

Vector2 Lane::getDirection() const {
    return { std::cos(heading), std::sin(heading) };
}

Vector2 Lane::getStartPoint() const {
    const Vector2 direction = getDirection();
    return { centerX - direction.x * length / 2.0f, centerY - direction.y * length / 2.0f };
}

Vector2 Lane::getEndPoint() const {
    const Vector2 direction = getDirection();
    return { centerX + direction.x * length / 2.0f, centerY + direction.y * length / 2.0f };
}
