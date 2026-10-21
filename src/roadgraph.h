#pragma once

#include "raylib.h"

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

class Lane;
class Intersection;
class Road;

struct RoadGraphNode {
    Vector2 position;
    std::vector<int> outgoingEdges;
};

struct RoadGraphEdge {
    int startNode = -1;
    int endNode = -1;
    std::vector<Vector2> points;
    float routingPenalty = 0.0f;
};

class RoadGraph {
public:
    void addRoad(const Road& road, const std::vector<Intersection>& intersections);
    void addLaneTransition(
        const Lane* incomingLane,
        const Lane* outgoingLane,
        const Intersection& intersection
    );
    std::optional<Vector2> snapPointToRoad(Vector2 worldPoint, float maxDistance = 150.0f) const;
    std::vector<Vector2> findRoute(Vector2 start, Vector2 end) const;

private:
    struct BoundaryNode {
        int nodeIndex;
        std::size_t waypointIndex;
    };
    struct NearbyNode {
        int nodeIndex;
        float distance;
    };

    std::optional<BoundaryNode> findLaneBoundaryNode(
        const std::vector<int>& candidates,
        const Lane& lane,
        Vector2 junctionCenter,
        Rectangle junctionBounds,
        bool incomingBoundary
    ) const;
    std::vector<NearbyNode> findNearbyNodes(Vector2 worldPoint, float maxDistance, float tieTolerance) const;
    std::optional<int> findNearestNode(Vector2 worldPoint, float maxDistance) const;

    static std::vector<Vector2> buildTransitionSpline(
        Vector2 entryPoint,
        Vector2 exitPoint,
        Vector2 incomingDirection,
        Vector2 outgoingDirection,
        int samples = 18,
        float handleRatio = 0.5f
    );
    static float distanceToRectangle(Vector2 point, Rectangle rect);

    std::vector<RoadGraphNode> nodes;
    std::vector<RoadGraphEdge> edges;
    std::unordered_map<const Lane*, std::vector<int>> laneNodes;
};
