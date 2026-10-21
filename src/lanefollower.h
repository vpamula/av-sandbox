#pragma once

#include "vehicle.h"
#include "raylib.h"
#include "trafficcontrol.h"

#include <cstddef>
#include <optional>
#include <vector>

class LaneFollower {
public:
    explicit LaneFollower(
        float cruiseSpeed = 200.0f,
        float lowTurnSpeed = 80.0f,
        float lookaheadDistance = 45.0f
    );

    void update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt);
    void reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints);
    void setRouteControls(std::vector<RouteControlPoint> controls);
    void setTrafficLightStates(const std::vector<TrafficLightState>& states);
    void setStopSignPermission(bool permitted);
    bool isWaitingAtStopSign() const;
    bool isWaitingAtTrafficLight() const;
    std::optional<std::size_t> getWaitingStopSignIntersectionIndex() const;

private:
    std::size_t currentWaypointIndex = 0;
    float cruiseSpeed;
    float lowTurnSpeed;
    float lookaheadDistance;
    std::vector<RouteControlPoint> routeControls;
    std::size_t currentControlIndex = 0;
    bool waitingAtControl = false;
    float stopWaitRemaining = 0.0f;
    bool stopSignPermission = false;
    std::vector<TrafficLightState> trafficLightStates;
};
