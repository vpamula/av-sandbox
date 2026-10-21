#include "lanefollower.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float WAYPOINT_REACHED_DISTANCE = 80.0f;
constexpr float LOW_TURN_SPEED = 80.0f;
constexpr float CRUISE_SPEED = 200.0f;
constexpr float MAX_STEERING_COMMAND = 0.8f;
}

LaneFollower::LaneFollower() = default;

void LaneFollower::reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints) {
    currentWaypointIndex = 0;
    if (waypoints.empty()) {
        return;
    }

    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
    float closestDistance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0; index < waypoints.size(); ++index) {
        const float distance = Vector2Distance(vehiclePosition, waypoints[index]);
        if (distance < closestDistance) {
            closestDistance = distance;
            currentWaypointIndex = index;
        }
    }
}

void LaneFollower::update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt) {
    if (waypoints.empty() || dt <= 0.0f) {
        vehicle.setSpeed(0.0f);
        vehicle.setSteeringAngle(0.0f);
        return;
    }

    const std::size_t lookaheadIndex = std::min(
        currentWaypointIndex + lookaheadPoints,
        waypoints.size() - 1
    );
    const Vector2 target = waypoints[lookaheadIndex];
    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
    const float desiredHeading = std::atan2(target.y - vehiclePosition.y, target.x - vehiclePosition.x);
    const float headingError = std::remainder(desiredHeading - vehicle.getHeading(), 2.0f * PI);
    const float absHeadingError = std::abs(headingError);

    vehicle.setSpeed(absHeadingError > 0.1f ? LOW_TURN_SPEED : CRUISE_SPEED);
    vehicle.setSteeringAngle(std::clamp(
        headingError * steeringGain,
        -MAX_STEERING_COMMAND,
        MAX_STEERING_COMMAND
    ));

    if (Vector2Distance(vehiclePosition, target) < WAYPOINT_REACHED_DISTANCE &&
        currentWaypointIndex + 1 < waypoints.size()) {
        ++currentWaypointIndex;
    }
}
