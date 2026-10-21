#pragma once

#include "lane.h"
#include "raylib.h"
#include "trafficcontrol.h"

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

class Road;

class Intersection {
public:
    Intersection(Rectangle bounds, std::vector<Vector2> surfacePolygon);

    Rectangle getBounds() const;
    bool containsPoint(Vector2 point) const;
    bool containsPointWithinDistance(Vector2 point, float distance) const;
    std::vector<std::vector<Vector2>> getPedestrianCrossings(float sidewalkExtension) const;
    void setSidewalkCorners(std::vector<std::array<Vector2, 4>> corners);
    void Draw() const;
    void setStopControlled(bool enabled);
    bool isStopControlled() const;
    void setTrafficLightControlled(bool enabled, float phaseOffsetSeconds = 0.0f);
    bool isTrafficLightControlled() const;
    void updateTrafficLight(float dt);
    TrafficLightAspect getTrafficLightAspect(bool horizontalAxis) const;
    std::optional<RouteControlPoint> getRouteControlAlongRoute(
        const std::vector<Vector2>& route,
        float vehicleLength
    ) const;
    void addLane(Lane* lane, const Road* road);
    const Road* getRoadForLane(const Lane* lane) const;
    const std::vector<Lane*>& getConnectedLanes() const;
    void computeTransitions();
    const std::vector<Lane*>& getNextLanes(const Lane* incomingLane) const;

private:
    struct StopApproach {
        Vector2 lineCenter;
        Vector2 lineStart;
        Vector2 lineEnd;
        Vector2 direction;
        const Road* road;
    };

    bool isValidTransition(const Lane* incomingLane, const Lane* outgoingLane) const;
    void rebuildStopApproaches();

    Rectangle bounds;
    std::vector<Vector2> surfacePolygon;
    std::vector<std::array<Vector2, 4>> sidewalkCorners;
    std::vector<Lane*> connectedLanes;
    std::unordered_map<const Lane*, std::vector<Lane*>> transitions;
    std::unordered_map<const Lane*, const Road*> laneRoads;
    bool stopControlled = false;
    bool trafficLightControlled = false;
    float trafficLightElapsed = 0.0f;
    std::vector<StopApproach> stopApproaches;
    std::vector<StopApproach> controlApproaches;
};
