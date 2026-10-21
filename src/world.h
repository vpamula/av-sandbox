#pragma once

#include "intersection.h"
#include "road.h"
#include "roadgraph.h"

#include <optional>
#include <vector>

class World {
public:
    static World makeDowntownGrid();

    void addRoad(Road road);
    void computeIntersections();
    void Draw() const;
    void updateTrafficLights(float dt);

    const std::vector<Road>& getRoads() const;
    std::optional<Vector2> findNearestRoadPoint(Vector2 worldPoint, float maxDistance = 200.0f) const;
    bool isPointInIntersection(Vector2 worldPoint) const;
    bool hasRoadCorridorLineOfSight(Vector2 from, Vector2 to) const;
    std::vector<Vector2> buildRouteBetweenPoints(Vector2 start, Vector2 end) const;
    std::vector<std::vector<Vector2>> getPedestrianCrossings() const;
    std::optional<Rectangle> getIntersectionBounds(std::size_t intersectionIndex) const;
    std::vector<RouteControlPoint> getRouteControlsAlongRoute(
        const std::vector<Vector2>& route,
        float vehicleLength
    ) const;
    std::vector<TrafficLightState> getTrafficLightStates() const;

private:
    void rebuildGraph();

    std::vector<Road> roads;
    std::vector<Intersection> intersections;
    RoadGraph graph;
};
