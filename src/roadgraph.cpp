#include "roadgraph.h"

#include "lane.h"
#include "raymath.h"
#include "road.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace {
constexpr float TRANSITION_SEARCH_RADIUS = 220.0f;

Vector2 normalizeOrZero(Vector2 value) {
    const float lengthSquared = value.x * value.x + value.y * value.y;
    if (lengthSquared <= 0.0001f) {
        return { 0.0f, 0.0f };
    }
    return Vector2Scale(value, 1.0f / std::sqrt(lengthSquared));
}
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
    const Vector2 cp1 = Vector2Add(entryPoint, Vector2Scale(incomingDir, travelLength));
    const Vector2 cp2 = Vector2Subtract(exitPoint, Vector2Scale(outgoingDir, travelLength));

    std::vector<Vector2> points;
    points.reserve(static_cast<std::size_t>(samples) + 1);

    for (int index = 0; index <= samples; ++index) {
        const float t = static_cast<float>(index) / static_cast<float>(samples);
        const float oneMinusT = 1.0f - t;
        const float startWeight = oneMinusT * oneMinusT * oneMinusT;
        const float firstControlWeight = 3.0f * oneMinusT * oneMinusT * t;
        const float secondControlWeight = 3.0f * oneMinusT * t * t;
        const float endWeight = t * t * t;

        points.push_back({
            startWeight * entryPoint.x + firstControlWeight * cp1.x +
                secondControlWeight * cp2.x + endWeight * exitPoint.x,
            startWeight * entryPoint.y + firstControlWeight * cp1.y +
                secondControlWeight * cp2.y + endWeight * exitPoint.y
        });
    }

    return points;
}

float RoadGraph::distanceToRectangle(Vector2 point, Rectangle rect) {
    const float closestX = std::clamp(point.x, rect.x, rect.x + rect.width);
    const float closestY = std::clamp(point.y, rect.y, rect.y + rect.height);
    return Vector2Distance(point, { closestX, closestY });
}

void RoadGraph::addRoad(const Road& road) {
    for (const Lane* lane : { &road.getRightLane(), &road.getLeftLane() }) {
        const std::vector<Vector2>& lanePoints = lane->getWaypoints();
        if (lanePoints.size() < 2) {
            continue;
        }

        std::vector<int>& laneNodeIndices = laneNodes[lane];
        laneNodeIndices.reserve(lanePoints.size());

        for (const Vector2 point : lanePoints) {
            const int nodeIndex = static_cast<int>(nodes.size());
            nodes.push_back({ point, {} });
            laneNodeIndices.push_back(nodeIndex);
        }

        for (std::size_t index = 0; index + 1 < laneNodeIndices.size(); ++index) {
            RoadGraphEdge edge;
            edge.startNode = laneNodeIndices[index];
            edge.endNode = laneNodeIndices[index + 1];
            edge.points = { lanePoints[index], lanePoints[index + 1] };

            const int edgeIndex = static_cast<int>(edges.size());
            edges.push_back(std::move(edge));
            nodes[static_cast<std::size_t>(laneNodeIndices[index])].outgoingEdges.push_back(edgeIndex);
        }
    }
}

std::optional<RoadGraph::BoundaryNode> RoadGraph::findLaneBoundaryNode(
    const std::vector<int>& candidates,
    const Lane& lane,
    Vector2 junctionCenter,
    Rectangle junctionBounds,
    bool incomingBoundary
) const {
    std::optional<BoundaryNode> bestNode;
    const float boundaryDistance = std::max(junctionBounds.width, junctionBounds.height) * 0.5f + 20.0f;
    float bestProjection = incomingBoundary
        ? -std::numeric_limits<float>::infinity()
        : std::numeric_limits<float>::infinity();

    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const int nodeIndex = candidates[index];
        const Vector2 point = nodes[static_cast<std::size_t>(nodeIndex)].position;
        if (distanceToRectangle(point, junctionBounds) > TRANSITION_SEARCH_RADIUS) {
            continue;
        }

        const Vector2 direction = lane.getDirectionAt(index);
        const Vector2 offset = Vector2Subtract(point, junctionCenter);
        const float projection = Vector2DotProduct(offset, direction);
        const bool onCorrectSide = incomingBoundary
            ? projection <= -boundaryDistance
            : projection >= boundaryDistance;
        if (!onCorrectSide) {
            continue;
        }

        const bool closerToJunction = incomingBoundary
            ? projection > bestProjection
            : projection < bestProjection;
        if (closerToJunction) {
            bestNode = BoundaryNode{ nodeIndex, index };
            bestProjection = projection;
        }
    }

    return bestNode;
}

void RoadGraph::addLaneTransition(
    const Lane* incomingLane,
    const Lane* outgoingLane,
    Rectangle junctionBounds
) {
    if (!incomingLane || !outgoingLane) {
        return;
    }

    const auto incomingIt = laneNodes.find(incomingLane);
    const auto outgoingIt = laneNodes.find(outgoingLane);
    if (incomingIt == laneNodes.end() || outgoingIt == laneNodes.end()) {
        return;
    }

    const Vector2 junctionCenter = {
        junctionBounds.x + junctionBounds.width * 0.5f,
        junctionBounds.y + junctionBounds.height * 0.5f
    };
    const auto incomingBoundary = findLaneBoundaryNode(
        incomingIt->second,
        *incomingLane,
        junctionCenter,
        junctionBounds,
        true
    );
    const auto outgoingBoundary = findLaneBoundaryNode(
        outgoingIt->second,
        *outgoingLane,
        junctionCenter,
        junctionBounds,
        false
    );
    if (!incomingBoundary || !outgoingBoundary) {
        return;
    }

    const Vector2 entryPoint = nodes[static_cast<std::size_t>(incomingBoundary->nodeIndex)].position;
    const Vector2 exitPoint = nodes[static_cast<std::size_t>(outgoingBoundary->nodeIndex)].position;
    RoadGraphEdge edge;
    edge.startNode = incomingBoundary->nodeIndex;
    edge.endNode = outgoingBoundary->nodeIndex;
    edge.points = buildTransitionSpline(
        entryPoint,
        exitPoint,
        incomingLane->getDirectionAt(incomingBoundary->waypointIndex),
        outgoingLane->getDirectionAt(outgoingBoundary->waypointIndex)
    );

    const int edgeIndex = static_cast<int>(edges.size());
    edges.push_back(std::move(edge));
    nodes[static_cast<std::size_t>(incomingBoundary->nodeIndex)].outgoingEdges.push_back(edgeIndex);
}

std::optional<Vector2> RoadGraph::snapPointToRoad(Vector2 worldPoint, float maxDistance) const {
    const auto nodeIndex = findNearestNode(worldPoint, maxDistance);
    return nodeIndex
        ? std::optional<Vector2>(nodes[static_cast<std::size_t>(*nodeIndex)].position)
        : std::nullopt;
}

std::optional<int> RoadGraph::findNearestNode(Vector2 worldPoint, float maxDistance) const {
    std::optional<int> best;
    float bestDistance = maxDistance;

    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const float distance = Vector2Distance(nodes[index].position, worldPoint);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = static_cast<int>(index);
        }
    }

    return best;
}

std::vector<RoadGraph::NearbyNode> RoadGraph::findNearbyNodes(
    Vector2 worldPoint,
    float maxDistance,
    float tieTolerance
) const {
    std::vector<NearbyNode> nearby;
    float closestDistance = maxDistance;

    for (const RoadGraphNode& node : nodes) {
        closestDistance = std::min(closestDistance, Vector2Distance(node.position, worldPoint));
    }
    if (closestDistance >= maxDistance) {
        return nearby;
    }

    const float maximumCandidateDistance = std::min(maxDistance, closestDistance + tieTolerance);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const float distance = Vector2Distance(nodes[index].position, worldPoint);
        if (distance <= maximumCandidateDistance) {
            nearby.push_back({ static_cast<int>(index), distance });
        }
    }

    return nearby;
}

std::vector<Vector2> RoadGraph::findRoute(Vector2 start, Vector2 end) const {
    const std::vector<NearbyNode> startCandidates = findNearbyNodes(start, 200.0f, 12.0f);
    const std::vector<NearbyNode> endCandidates = findNearbyNodes(end, 200.0f, 12.0f);
    if (startCandidates.empty() || endCandidates.empty()) {
        return {};
    }

    std::vector<float> cost(nodes.size(), std::numeric_limits<float>::infinity());
    std::vector<int> previous(nodes.size(), -1);
    std::vector<int> previousEdge(nodes.size(), -1);
    std::vector<float> endSnapDistance(nodes.size(), std::numeric_limits<float>::infinity());
    for (const NearbyNode& candidate : startCandidates) {
        const std::size_t nodeIndex = static_cast<std::size_t>(candidate.nodeIndex);
        if (candidate.distance < cost[nodeIndex]) {
            cost[nodeIndex] = candidate.distance;
        }
    }
    for (const NearbyNode& candidate : endCandidates) {
        endSnapDistance[static_cast<std::size_t>(candidate.nodeIndex)] = candidate.distance;
    }

    using FrontierEntry = std::pair<float, int>;
    std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, std::greater<FrontierEntry>> frontier;

    for (const NearbyNode& candidate : startCandidates) {
        frontier.push({ candidate.distance, candidate.nodeIndex });
    }
    float bestRouteCost = std::numeric_limits<float>::infinity();
    int bestEndNode = -1;

    while (!frontier.empty()) {
        const auto [currentCost, currentNode] = frontier.top();
        frontier.pop();

        if (currentCost > cost[static_cast<std::size_t>(currentNode)]) {
            continue;
        }
        if (currentCost >= bestRouteCost) {
            break;
        }

        const float endpointDistance = endSnapDistance[static_cast<std::size_t>(currentNode)];
        if (std::isfinite(endpointDistance) && currentCost + endpointDistance < bestRouteCost) {
            bestRouteCost = currentCost + endpointDistance;
            bestEndNode = currentNode;
        }

        for (const int edgeIndex : nodes[static_cast<std::size_t>(currentNode)].outgoingEdges) {
            const RoadGraphEdge& edge = edges[static_cast<std::size_t>(edgeIndex)];
            if (edge.startNode != currentNode) {
                continue;
            }

            float edgeLength = 0.0f;
            for (std::size_t index = 1; index < edge.points.size(); ++index) {
                edgeLength += Vector2Distance(edge.points[index - 1], edge.points[index]);
            }

            const std::size_t nextNode = static_cast<std::size_t>(edge.endNode);
            const float nextCost = currentCost + edgeLength;
            if (nextCost < cost[nextNode]) {
                cost[nextNode] = nextCost;
                previous[nextNode] = currentNode;
                previousEdge[nextNode] = edgeIndex;
                frontier.push({ nextCost, edge.endNode });
            }
        }
    }

    if (bestEndNode < 0) {
        return {};
    }

    std::vector<int> pathEdges;
    int currentNode = bestEndNode;
    while (previousEdge[static_cast<std::size_t>(currentNode)] >= 0) {
        const int edgeIndex = previousEdge[static_cast<std::size_t>(currentNode)];
        pathEdges.push_back(edgeIndex);
        currentNode = previous[static_cast<std::size_t>(currentNode)];
    }
    std::reverse(pathEdges.begin(), pathEdges.end());

    std::vector<Vector2> route;
    route.push_back(nodes[static_cast<std::size_t>(currentNode)].position);
    for (const int edgeIndex : pathEdges) {
        const RoadGraphEdge& edge = edges[static_cast<std::size_t>(edgeIndex)];
        for (std::size_t index = 1; index < edge.points.size(); ++index) {
            route.push_back(edge.points[index]);
        }
    }

    return route;
}
