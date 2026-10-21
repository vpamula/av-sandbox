#pragma once

#include <cstddef>

enum class RouteControlType {
    StopSign,
    TrafficLight
};

enum class TrafficLightAspect {
    Red,
    Yellow,
    Green
};

struct TrafficLightState {
    TrafficLightAspect horizontal = TrafficLightAspect::Green;
    TrafficLightAspect vertical = TrafficLightAspect::Green;
};

struct RouteControlPoint {
    float distance = 0.0f;
    RouteControlType type = RouteControlType::StopSign;
    bool horizontalAxis = false;
    std::size_t intersectionIndex = 0;
};
