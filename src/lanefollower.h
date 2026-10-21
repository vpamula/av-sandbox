#pragma once

#include "vehicle.h"
#include "raylib.h"
#include "trafficcontrol.h"

#include <cstddef>
#include <vector>

class LaneFollower {
public:
    LaneFollower();

    void update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt);
    void reacquireWaypoint(const Vehicle& vehicle, const std::vector<Vector2>& waypoints);
    void setRouteControls(std::vector<RouteControlPoint> controls);
    void setTrafficLightAspects(TrafficLightAspect horizontal, TrafficLightAspect vertical);
    bool isWaitingAtStopSign() const;
    bool isWaitingAtTrafficLight() const;

private:
    std::size_t currentWaypointIndex = 0;
    std::vector<RouteControlPoint> routeControls;
    std::size_t currentControlIndex = 0;
    bool waitingAtControl = false;
    float stopWaitRemaining = 0.0f;
    TrafficLightAspect horizontalSignal = TrafficLightAspect::Green;
    TrafficLightAspect verticalSignal = TrafficLightAspect::Green;
};
