#pragma once

enum class RouteControlType {
    StopSign,
    TrafficLight
};

enum class TrafficLightAspect {
    Red,
    Yellow,
    Green
};

struct RouteControlPoint {
    float distance = 0.0f;
    RouteControlType type = RouteControlType::StopSign;
    bool horizontalAxis = false;
};
