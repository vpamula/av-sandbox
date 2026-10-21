#include "lanefollower.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr float MAX_STEERING_COMMAND = 0.87f;
constexpr float STOPPING_DECELERATION = 360.0f;
constexpr float STOP_POSITION_TOLERANCE = 12.0f;
constexpr float STOP_DWELL_SECONDS = 1.25f;
constexpr float AUTONOMOUS_ACCELERATION = 180.0f;
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

float routeDistanceAt(
    const std::vector<Vector2>& waypoints,
    std::size_t segmentIndex,
    float amount
) {
    float distance = 0.0f;
    for (std::size_t index = 0; index < segmentIndex; ++index) {
        distance += Vector2Distance(waypoints[index], waypoints[index + 1]);
    }

    return distance + Vector2Distance(waypoints[segmentIndex], waypoints[segmentIndex + 1]) * amount;
}
}

LaneFollower::LaneFollower(float cruiseSpeed, float lowTurnSpeed, float lookaheadDistance)
    : cruiseSpeed(cruiseSpeed),
      lowTurnSpeed(lowTurnSpeed),
      lookaheadDistance(lookaheadDistance) {}

void LaneFollower::setRouteControls(std::vector<RouteControlPoint> controls) {
    std::sort(controls.begin(), controls.end(), [](const RouteControlPoint& first, const RouteControlPoint& second) {
        return first.distance < second.distance;
    });
    routeControls = std::move(controls);
    currentControlIndex = 0;
    waitingAtControl = false;
    stopWaitRemaining = 0.0f;
    stopSignPermission = false;
}

void LaneFollower::setTrafficLightStates(const std::vector<TrafficLightState>& states) {
    trafficLightStates = states;
}

void LaneFollower::setStopSignPermission(bool permitted) {
    stopSignPermission = permitted;
}

bool LaneFollower::isWaitingAtStopSign() const {
    return waitingAtControl && currentControlIndex < routeControls.size() &&
        routeControls[currentControlIndex].type == RouteControlType::StopSign;
}

bool LaneFollower::isWaitingAtTrafficLight() const {
    return waitingAtControl && currentControlIndex < routeControls.size() &&
        routeControls[currentControlIndex].type == RouteControlType::TrafficLight;
}

std::optional<std::size_t> LaneFollower::getWaitingStopSignIntersectionIndex() const {
    if (!isWaitingAtStopSign()) {
        return std::nullopt;
    }
    return routeControls[currentControlIndex].intersectionIndex;
}

void LaneFollower::reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints) {
    currentWaypointIndex = 0;
    if (waypoints.size() < 2) {
        return;
    }

    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
    float closestDistanceSquared = std::numeric_limits<float>::infinity();
    SegmentProjection closestProjection{};
    for (std::size_t index = 0; index + 1 < waypoints.size(); ++index) {
        const SegmentProjection projection = projectOntoSegment(
            vehiclePosition,
            waypoints[index],
            waypoints[index + 1]
        );
        if (projection.distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = projection.distanceSquared;
            currentWaypointIndex = index;
            closestProjection = projection;
        }
    }

    if (!waitingAtControl) {
        const float progress = routeDistanceAt(waypoints, currentWaypointIndex, closestProjection.amount);
        while (currentControlIndex < routeControls.size() &&
               routeControls[currentControlIndex].distance < progress - STOP_POSITION_TOLERANCE) {
            ++currentControlIndex;
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

    const float routeProgress = routeDistanceAt(waypoints, closestSegment, closestProjection.amount);
    RouteControlPoint* control = currentControlIndex < routeControls.size()
        ? &routeControls[currentControlIndex]
        : nullptr;
    float distanceToControl = control
        ? control->distance - routeProgress
        : std::numeric_limits<float>::infinity();
    bool requiresStop = false;

    auto aspectAtControl = [&](const RouteControlPoint& routeControl) {
        if (routeControl.intersectionIndex >= trafficLightStates.size()) {
            return TrafficLightAspect::Green;
        }
        const TrafficLightState& state = trafficLightStates[routeControl.intersectionIndex];
        return routeControl.horizontalAxis ? state.horizontal : state.vertical;
    };

    if (control) {
        if (control->type == RouteControlType::StopSign) {
            requiresStop = true;
        }
        else {
            const TrafficLightAspect aspect = aspectAtControl(*control);
            if (aspect == TrafficLightAspect::Red) {
                requiresStop = true;
            }
            else if (aspect == TrafficLightAspect::Yellow) {
                const float stoppingDistance = vehicle.getSpeed() * vehicle.getSpeed() /
                    (2.0f * STOPPING_DECELERATION);
                requiresStop = distanceToControl > stoppingDistance + STOP_POSITION_TOLERANCE;
            }

            if (!waitingAtControl && !requiresStop &&
                distanceToControl < -STOP_POSITION_TOLERANCE) {
                ++currentControlIndex;
                control = nullptr;
                distanceToControl = std::numeric_limits<float>::infinity();
            }
        }
    }

    if (waitingAtControl && control) {
        if (control->type == RouteControlType::StopSign) {
            requiresStop = true;
            if (vehicle.getSpeed() <= 3.0f) {
                stopWaitRemaining = std::max(0.0f, stopWaitRemaining - dt);
                if (stopWaitRemaining <= 0.0f && stopSignPermission) {
                    waitingAtControl = false;
                    stopSignPermission = false;
                    ++currentControlIndex;
                    control = nullptr;
                    requiresStop = false;
                }
            }
        }
        else {
            const TrafficLightAspect aspect = aspectAtControl(*control);
            if (aspect == TrafficLightAspect::Green) {
                waitingAtControl = false;
                ++currentControlIndex;
                control = nullptr;
                requiresStop = false;
            }
            else {
                requiresStop = true;
            }
        }
    }
    else if (control && requiresStop && distanceToControl <= STOP_POSITION_TOLERANCE &&
             vehicle.getSpeed() <= 3.0f) {
        waitingAtControl = true;
        if (control->type == RouteControlType::StopSign) {
            stopWaitRemaining = STOP_DWELL_SECONDS;
        }
    }

    const float distanceUntilControl = requiresStop
        ? std::max(0.0f, distanceToControl)
        : std::numeric_limits<float>::infinity();
    Vector2 target = waypoints.back();
    float remainingDistance = requiresStop
        ? std::min(lookaheadDistance, std::max(distanceUntilControl, 1.0f))
        : lookaheadDistance;
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
    float targetSpeed = std::abs(targetAngle) > 0.12f ? lowTurnSpeed : cruiseSpeed;

    if (requiresStop) {
        if (waitingAtControl || distanceUntilControl <= STOP_POSITION_TOLERANCE) {
            targetSpeed = 0.0f;
        }
        else {
            const float brakingDistance = distanceUntilControl - STOP_POSITION_TOLERANCE;
            const float safeSpeed = std::sqrt(2.0f * STOPPING_DECELERATION * brakingDistance);
            targetSpeed = std::min(targetSpeed, safeSpeed);
        }
    }

    if (!routeControls.empty()) {
        vehicle.approachSpeed(
            targetSpeed,
            AUTONOMOUS_ACCELERATION,
            STOPPING_DECELERATION,
            dt
        );
    }
    else {
        vehicle.setSpeed(targetSpeed);
    }

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
