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
    float cost = 0.0f;
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

    std::vector<BoundaryNode> findLaneBoundaryNodes(
        const std::vector<int>& laneNodes,
        const Lane& lane,
        Vector2 intersectionCenter,
        Rectangle intersectionBounds,
        bool incoming
    ) const;
    std::optional<int> findNearestNode(Vector2 point, float maxDistance) const;
    void addEdge(int startNode, int endNode, std::vector<Vector2> points, float extraCost = 0.0f);

    static std::vector<Vector2> buildTransitionSpline(
        Vector2 entry,
        Vector2 exit,
        Vector2 incomingDirection,
        Vector2 outgoingDirection,
        int samples = 18,
        float handleRatio = 0.5f
    );

    std::vector<RoadGraphNode> nodes;
    std::vector<RoadGraphEdge> edges;
    std::unordered_map<const Lane*, std::vector<int>> laneNodes;
};
