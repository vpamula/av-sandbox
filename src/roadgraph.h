#pragma once

#include "raylib.h"

#include <optional>
#include <unordered_map>
#include <vector>

class Lane;
class Road;

struct RoadGraphNode {
    Vector2 position;
    std::vector<int> outgoingEdges;
};

struct RoadGraphEdge {
    int startNode = -1;
    int endNode = -1;
    std::vector<Vector2> points;
};

class RoadGraph {
public:
    void addRoad(const Road& road);
    void addLaneTransition(const Lane* incomingLane, const Lane* outgoingLane, Rectangle junctionBounds);
    std::optional<Vector2> snapPointToRoad(Vector2 worldPoint, float maxDistance = 150.0f) const;
    std::vector<Vector2> findRoute(Vector2 start, Vector2 end) const;

private:
    int findOrCreateNode(Vector2 position, float tolerance);
    void addLaneNode(const Lane* lane, int nodeIndex);
    std::optional<int> findNearestNode(Vector2 worldPoint, float maxDistance) const;

    static std::vector<Vector2> buildTransitionSpline(
        Vector2 entryPoint,
        Vector2 exitPoint,
        Vector2 incomingDirection,
        Vector2 outgoingDirection,
        int samples = 18
    );
    static float distanceToRectangle(Vector2 point, Rectangle rect);

    std::vector<RoadGraphNode> nodes;
    std::vector<RoadGraphEdge> edges;
    std::unordered_map<const Lane*, std::vector<int>> laneNodes;
};
