#pragma once

#include "lane.h"
#include "raylib.h"
#include "trafficcontrol.h"

#include <optional>
#include <unordered_map>
#include <vector>

class Intersection {
public:
    Intersection(Rectangle bounds, std::vector<Vector2> surfacePolygon);

    Rectangle getBounds() const;
    bool containsPoint(Vector2 point) const;
    void Draw() const;
    void setStopControlled(bool enabled);
    bool isStopControlled() const;
    void setTrafficLightControlled(bool enabled);
    bool isTrafficLightControlled() const;
    void updateTrafficLight(float dt);
    TrafficLightAspect getTrafficLightAspect(bool horizontalAxis) const;
    std::optional<RouteControlPoint> getRouteControlAlongRoute(
        const std::vector<Vector2>& route,
        float vehicleLength
    ) const;
    void addLane(Lane* lane);
    const std::vector<Lane*>& getConnectedLanes() const;
    void computeTransitions();
    const std::vector<Lane*>& getNextLanes(const Lane* incomingLane) const;

private:
    struct StopApproach {
        Vector2 lineCenter;
        Vector2 direction;
    };

    bool isValidTransition(const Lane* incomingLane, const Lane* outgoingLane) const;
    void rebuildStopApproaches();

    Rectangle bounds;
    std::vector<Vector2> surfacePolygon;
    std::vector<Lane*> connectedLanes;
    std::unordered_map<const Lane*, std::vector<Lane*>> transitions;
    bool stopControlled = false;
    bool trafficLightControlled = false;
    float trafficLightElapsed = 0.0f;
    std::vector<StopApproach> stopApproaches;
};
