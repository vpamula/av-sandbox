#include "lanefollower.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float LOOKAHEAD_DISTANCE = 45.0f;
constexpr float LOW_TURN_SPEED = 80.0f;
constexpr float CRUISE_SPEED = 200.0f;
constexpr float MAX_STEERING_COMMAND = 0.87f;
constexpr std::size_t SEGMENTS_TO_SEARCH_AHEAD = 12;

struct SegmentProjection {
    Vector2 point;
    float amount;
    float distanceSquared;
};

SegmentProjection projectOntoSegment(Vector2 point, Vector2 start, Vector2 end) {
    const Vector2 segment = Vector2Subtract(end, start);
    const float segmentLengthSquared = Vector2LengthSqr(segment);
    const float amount = segmentLengthSquared <= 0.0001f
        ? 0.0f
        : std::clamp(
            Vector2DotProduct(Vector2Subtract(point, start), segment) / segmentLengthSquared,
            0.0f,
            1.0f
        );
    const Vector2 projected = Vector2Add(start, Vector2Scale(segment, amount));
    return { projected, amount, Vector2LengthSqr(Vector2Subtract(point, projected)) };
}

Vector2 interpolate(Vector2 start, Vector2 end, float amount) {
    return Vector2Add(start, Vector2Scale(Vector2Subtract(end, start), amount));
}
}

LaneFollower::LaneFollower() = default;

void LaneFollower::reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints) {
    currentWaypointIndex = 0;
    if (waypoints.size() < 2) {
        return;
    }

    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
    float closestDistanceSquared = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0; index + 1 < waypoints.size(); ++index) {
        const SegmentProjection projection = projectOntoSegment(
            vehiclePosition,
            waypoints[index],
            waypoints[index + 1]
        );
        if (projection.distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = projection.distanceSquared;
            currentWaypointIndex = index;
        }
    }
}

void LaneFollower::update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt) {
    if (waypoints.size() < 2 || dt <= 0.0f) {
        vehicle.setSpeed(0.0f);
        vehicle.setSteeringAngle(0.0f);
        return;
    }

    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
    const std::size_t lastSegment = waypoints.size() - 2;
    currentWaypointIndex = std::min(currentWaypointIndex, lastSegment);
    const std::size_t searchEnd = std::min(lastSegment, currentWaypointIndex + SEGMENTS_TO_SEARCH_AHEAD);

    std::size_t closestSegment = currentWaypointIndex;
    SegmentProjection closestProjection = projectOntoSegment(
        vehiclePosition,
        waypoints[closestSegment],
        waypoints[closestSegment + 1]
    );

    for (std::size_t index = currentWaypointIndex + 1; index <= searchEnd; ++index) {
        const SegmentProjection projection = projectOntoSegment(
            vehiclePosition,
            waypoints[index],
            waypoints[index + 1]
        );
        if (projection.distanceSquared < closestProjection.distanceSquared) {
            closestProjection = projection;
            closestSegment = index;
        }
    }
    currentWaypointIndex = closestSegment;

    Vector2 target = waypoints.back();
    float remainingDistance = LOOKAHEAD_DISTANCE;
    Vector2 segmentStart = closestProjection.point;

    for (std::size_t index = closestSegment; index + 1 < waypoints.size(); ++index) {
        const Vector2 segmentEnd = waypoints[index + 1];
        const float segmentLength = Vector2Distance(segmentStart, segmentEnd);
        if (segmentLength >= remainingDistance) {
            target = interpolate(segmentStart, segmentEnd, remainingDistance / segmentLength);
            break;
        }

        remainingDistance -= segmentLength;
        segmentStart = segmentEnd;
    }

    const float heading = vehicle.getHeading();
    const Vector2 targetOffset = Vector2Subtract(target, vehiclePosition);
    const Vector2 localTarget = {
        std::cos(heading) * targetOffset.x + std::sin(heading) * targetOffset.y,
        -std::sin(heading) * targetOffset.x + std::cos(heading) * targetOffset.y
    };
    const float targetDistanceSquared = std::max(Vector2LengthSqr(localTarget), 1.0f);
    const float curvature = 2.0f * localTarget.y / targetDistanceSquared;
    const float targetAngle = std::atan2(localTarget.y, localTarget.x);
    const float targetSpeed = std::abs(targetAngle) > 0.12f ? LOW_TURN_SPEED : CRUISE_SPEED;
    vehicle.setSpeed(targetSpeed);

    // Match the wheel angle to the vehicle's speed-dependent bicycle model.
    const float speedSquared = targetSpeed * targetSpeed;
    const float effectiveWheelbase = vehicle.getLength() * (1.0f + 0.00001f * speedSquared);
    const float steeringAngle = std::atan(effectiveWheelbase * curvature);
    vehicle.setSteeringAngle(std::clamp(
        steeringAngle,
        -MAX_STEERING_COMMAND,
        MAX_STEERING_COMMAND
    ));
}
