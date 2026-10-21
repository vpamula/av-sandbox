#include "roadgraph.h"
#include "lane.h"
#include "road.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace {
Vector2 normalizeOrZero(Vector2 value) {
    const float lengthSquared = value.x * value.x + value.y * value.y;
    if (lengthSquared <= 0.0001f) {
        return { 0.0f, 0.0f };
    }
    const float length = std::sqrt(lengthSquared);
    return { value.x / length, value.y / length };
}
}

int RoadGraph::findOrCreateNode(Vector2 position, float tolerance) {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (Vector2Distance(nodes[i].position, position) <= tolerance) {
            return static_cast<int>(i);
        }
    }

    RoadGraphNode node;
    node.position = position;
    nodes.push_back(node);
    return static_cast<int>(nodes.size() - 1);
}

float RoadGraph::distanceToRectangle(Vector2 point, Rectangle rect) {
    const float closestX = std::clamp(point.x, rect.x, rect.x + rect.width);
    const float closestY = std::clamp(point.y, rect.y, rect.y + rect.height);
    const float dx = point.x - closestX;
    const float dy = point.y - closestY;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Vector2> RoadGraph::buildTransitionSpline(
    Vector2 entryPoint,
    Vector2 exitPoint,
    Vector2 incomingDirection,
    Vector2 outgoingDirection,
    int samples
) {
    const Vector2 incomingDir = normalizeOrZero(incomingDirection);
    const Vector2 outgoingDir = normalizeOrZero(outgoingDirection);
    const float travelLength = std::max(40.0f, Vector2Distance(entryPoint, exitPoint) * 0.5f);
    const Vector2 cp1 = {
        entryPoint.x + incomingDir.x * travelLength,
        entryPoint.y + incomingDir.y * travelLength
    };
    const Vector2 cp2 = {
        exitPoint.x - outgoingDir.x * travelLength,
        exitPoint.y - outgoingDir.y * travelLength
    };

    std::vector<Vector2> points;
    points.reserve(static_cast<size_t>(samples) + 1);

    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples);
        const float oneMinusT = 1.0f - t;
        const Vector2 point = {
            oneMinusT * oneMinusT * oneMinusT * entryPoint.x +
                3.0f * oneMinusT * oneMinusT * t * cp1.x +
                3.0f * oneMinusT * t * t * cp2.x +
                t * t * t * exitPoint.x,
            oneMinusT * oneMinusT * oneMinusT * entryPoint.y +
                3.0f * oneMinusT * oneMinusT * t * cp1.y +
                3.0f * oneMinusT * t * t * cp2.y +
                t * t * t * exitPoint.y
        };
        points.push_back(point);
    }

    return points;
}

void RoadGraph::addLaneNode(const Lane* lane, int nodeIndex) {
    auto& nodesForLane = laneNodes[lane];
    if (std::find(nodesForLane.begin(), nodesForLane.end(), nodeIndex) == nodesForLane.end()) {
        nodesForLane.push_back(nodeIndex);
    }
}

void RoadGraph::addLaneTransition(const Lane* incomingLane, const Lane* outgoingLane, Rectangle junctionBounds) {
    if (!incomingLane || !outgoingLane) {
        return;
    }

    const auto incomingIt = laneNodes.find(incomingLane);
    const auto outgoingIt = laneNodes.find(outgoingLane);
    if (incomingIt == laneNodes.end() || outgoingIt == laneNodes.end()) {
        return;
    }

    // A turn can connect nodes near opposite corners of the junction. The
    // diagonal of the junction (plus one waypoint interval) is a safer lower
    // bound than a fixed 200-unit radius, which rejected the first left turn.
    const float junctionDiagonal = Vector2Distance(
        { junctionBounds.x, junctionBounds.y },
        { junctionBounds.x + junctionBounds.width, junctionBounds.y + junctionBounds.height }
    );
    const float transitionDistanceLimit = std::max(200.0f, junctionDiagonal + 100.0f);

    auto collectBoundaryNode = [&](const std::vector<int>& candidates, const Lane* lane, bool isIncomingBoundary) -> std::optional<int> {
        const Vector2 laneStart = lane->getStartPoint();
        const Vector2 laneDirection = normalizeOrZero(lane->getDirection());

        // Project the intersection rectangle onto the lane direction. The
        // nearest node before this interval is the approach point; the nearest
        // node after it is the departure point.
        const Vector2 corners[] = {
            { junctionBounds.x, junctionBounds.y },
            { junctionBounds.x + junctionBounds.width, junctionBounds.y },
            { junctionBounds.x, junctionBounds.y + junctionBounds.height },
            { junctionBounds.x + junctionBounds.width, junctionBounds.y + junctionBounds.height }
        };
        float minProjection = std::numeric_limits<float>::infinity();
        float maxProjection = -std::numeric_limits<float>::infinity();
        for (const Vector2 corner : corners) {
            const float projection = Vector2DotProduct(
                { corner.x - laneStart.x, corner.y - laneStart.y },
                laneDirection
            );
            minProjection = std::min(minProjection, projection);
            maxProjection = std::max(maxProjection, projection);
        }

        std::optional<int> bestNode;
        float bestProjection = isIncomingBoundary
            ? -std::numeric_limits<float>::infinity()
            : std::numeric_limits<float>::infinity();

        for (int nodeIndex : candidates) {
            const Vector2 point = nodes[nodeIndex].position;
            const float projection = Vector2DotProduct({ point.x - laneStart.x, point.y - laneStart.y }, laneDirection);
            const float distanceToBounds = distanceToRectangle(point, junctionBounds);
            if (distanceToBounds > 200.0f) {
                continue;
            }

            const bool onCorrectSide = isIncomingBoundary
                ? projection <= minProjection + 0.001f
                : projection >= maxProjection - 0.001f;
            if (!onCorrectSide) {
                continue;
            }

            const bool closerToIntersection = !bestNode || (isIncomingBoundary
                ? projection > bestProjection
                : projection < bestProjection);
            if (closerToIntersection) {
                bestNode = nodeIndex;
                bestProjection = projection;
            }
        }

        return bestNode;
    };

    const auto incomingBoundaryNode = collectBoundaryNode(incomingIt->second, incomingLane, true);
    const auto outgoingBoundaryNode = collectBoundaryNode(outgoingIt->second, outgoingLane, false);
    if (!incomingBoundaryNode || !outgoingBoundaryNode) {
        return;
    }

    const int incomingNode = *incomingBoundaryNode;
    const int outgoingNode = *outgoingBoundaryNode;
    const Vector2 entryPoint = nodes[incomingNode].position;
    const Vector2 exitPoint = nodes[outgoingNode].position;
    if (Vector2Distance(entryPoint, exitPoint) > transitionDistanceLimit) {
        return;
    }

    RoadGraphEdge edge;
    edge.startNode = incomingNode;
    edge.endNode = outgoingNode;
    edge.points = buildTransitionSpline(
        entryPoint,
        exitPoint,
        incomingLane->getDirection(),
        outgoingLane->getDirection(),
        18
    );

    const int edgeIndex = static_cast<int>(edges.size());
    edges.push_back(edge);
    nodes[incomingNode].outgoingEdges.push_back(edgeIndex);
}

void RoadGraph::addRoad(const Road& road) {
    for (bool isRightLane : { true, false }) {
        const Lane& lane = isRightLane ? road.getRightLane() : road.getLeftLane();
        const std::vector<Vector2> lanePoints = lane.generateWaypoints();

        if (lanePoints.size() < 2) {
            continue;
        }

        std::vector<int> laneNodeIndices;
        laneNodeIndices.reserve(lanePoints.size());

        for (const Vector2& point : lanePoints) {
            const int nodeIndex = findOrCreateNode(point, 20.0f);
            addLaneNode(&lane, nodeIndex);
            laneNodeIndices.push_back(nodeIndex);
        }

        for (size_t i = 0; i + 1 < laneNodeIndices.size(); ++i) {
            const int startNode = laneNodeIndices[i];
            const int endNode = laneNodeIndices[i + 1];

            RoadGraphEdge edge;
            edge.startNode = startNode;
            edge.endNode = endNode;
            edge.points = { lanePoints[i], lanePoints[i + 1] };

            const int edgeIndex = static_cast<int>(edges.size());
            edges.push_back(edge);
            nodes[startNode].outgoingEdges.push_back(edgeIndex);
        }
    }
}

std::optional<Vector2> RoadGraph::snapPointToRoad(Vector2 worldPoint, float maxDistance) const {
    const auto nodeIndex = findNearestNode(worldPoint, maxDistance);
    return nodeIndex ? std::optional<Vector2>(nodes[*nodeIndex].position) : std::nullopt;
}

std::optional<int> RoadGraph::findNearestNode(Vector2 worldPoint, float maxDistance) const {
    std::optional<int> best;
    float bestDistance = maxDistance;

    for (size_t i = 0; i < nodes.size(); ++i) {
        const float distance = Vector2Distance(nodes[i].position, worldPoint);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = static_cast<int>(i);
        }
    }

    return best;
}

std::vector<Vector2> RoadGraph::findRoute(Vector2 start, Vector2 end) const {
    const auto startNodeOpt = findNearestNode(start, 200.0f);
    const auto endNodeOpt = findNearestNode(end, 200.0f);
    if (!startNodeOpt || !endNodeOpt) {
        return {};
    }

    const int startNode = *startNodeOpt;
    const int endNode = *endNodeOpt;
    if (startNode == endNode) {
        return { nodes[startNode].position };
    }

    std::vector<float> cost(nodes.size(), std::numeric_limits<float>::infinity());
    std::vector<int> previous(nodes.size(), -1);
    std::vector<int> previousEdge(nodes.size(), -1);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> frontier;

    cost[startNode] = 0.0f;
    frontier.push({ 0.0f, startNode });

    while (!frontier.empty()) {
        const auto [currentCost, currentNode] = frontier.top();
        frontier.pop();

        if (currentCost > cost[currentNode]) {
            continue;
        }

        if (currentNode == endNode) {
            break;
        }

        for (int edgeIndex : nodes[currentNode].outgoingEdges) {
            const RoadGraphEdge& edge = edges[edgeIndex];
            // Edges represent lane travel direction. Traversing them backward
            // would let routes drive against traffic (and reverse transitions).
            if (edge.startNode != currentNode) {
                continue;
            }
            const int nextNode = edge.endNode;

            float edgeLength = 0.0f;
            for (size_t i = 1; i < edge.points.size(); ++i) {
                edgeLength += Vector2Distance(edge.points[i - 1], edge.points[i]);
            }

            const float nextCost = currentCost + edgeLength;
            if (nextCost < cost[nextNode]) {
                cost[nextNode] = nextCost;
                previous[nextNode] = currentNode;
                previousEdge[nextNode] = edgeIndex;
                frontier.push({ nextCost, nextNode });
            }
        }
    }

    if (cost[endNode] == std::numeric_limits<float>::infinity()) {
        return {};
    }

    std::vector<int> pathEdges;
    int current = endNode;
    while (current != startNode) {
        if (current < 0 || previousEdge[current] < 0) {
            return {};
        }
        pathEdges.push_back(previousEdge[current]);
        current = previous[current];
    }
    std::reverse(pathEdges.begin(), pathEdges.end());

    std::vector<Vector2> route;
    route.push_back(nodes[startNode].position);
    for (int edgeIndex : pathEdges) {
        const RoadGraphEdge& edge = edges[edgeIndex];
        // Include transition spline samples so the vehicle follows the actual
        // turn geometry instead of cutting across the junction in a straight line.
        for (size_t i = 1; i < edge.points.size(); ++i) {
            route.push_back(edge.points[i]);
        }
    }

    return route;
}
