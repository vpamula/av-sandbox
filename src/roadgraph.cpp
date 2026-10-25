#include "roadgraph.h"

#include "intersection.h"
#include "lane.h"
#include "raymath.h"
#include "road.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace {
constexpr float TRANSITION_SEARCH_RADIUS = 220.0f;
constexpr float LANE_CHANGE_LENGTH = 120.0f;
constexpr float LANE_CHANGE_SPACING = 120.0f;
constexpr int LANE_CHANGE_SAMPLES = 12;
constexpr float BIKE_LANE_CLEARANCE = 36.0f;
constexpr float CROSSWALK_CLEARANCE = 48.0f;
constexpr float MAX_INTERSECTION_PENALTY_RATIO = 0.03f;
constexpr float TURN_HANDLE_RATIO = 0.58f;

float polylineLength(const std::vector<Vector2>& points) {
    float length = 0.0f;
    for (std::size_t index = 1; index < points.size(); ++index) {
        length += Vector2Distance(points[index - 1], points[index]);
    }
    return length;
}

Vector2 normalizeOrZero(Vector2 value) {
    const float length = Vector2Length(value);
    return length > 0.0001f ? Vector2Scale(value, 1.0f / length) : Vector2{ 0.0f, 0.0f };
}

std::size_t waypointIndexAtDistance(const std::vector<Vector2>& points, float targetDistance) {
    if (points.size() < 2 || targetDistance <= 0.0f) {
        return 0;
    }

    std::size_t bestIndex = 0;
    float bestDifference = targetDistance;
    float distance = 0.0f;
    for (std::size_t index = 1; index < points.size(); ++index) {
        distance += Vector2Distance(points[index - 1], points[index]);
        const float difference = std::abs(distance - targetDistance);
        if (difference < bestDifference) {
            bestDifference = difference;
            bestIndex = index;
        }
    }
    return bestIndex;
}

float distanceToSegment(Vector2 point, Vector2 start, Vector2 end) {
    const Vector2 segment = Vector2Subtract(end, start);
    const float lengthSquared = Vector2LengthSqr(segment);
    if (lengthSquared <= 0.0001f) {
        return Vector2Distance(point, start);
    }

    const float projection = std::clamp(
        Vector2DotProduct(Vector2Subtract(point, start), segment) / lengthSquared,
        0.0f,
        1.0f
    );
    const Vector2 closest = Vector2Add(start, Vector2Scale(segment, projection));
    return Vector2Distance(point, closest);
}

float distanceToPolyline(Vector2 point, const std::vector<Vector2>& line) {
    float closest = std::numeric_limits<float>::infinity();
    for (std::size_t index = 1; index < line.size(); ++index) {
        closest = std::min(closest, distanceToSegment(point, line[index - 1], line[index]));
    }
    return closest;
}

float nearbyPathCost(
    const std::vector<Vector2>& route,
    const std::vector<const std::vector<Vector2>*>& markedPaths,
    float clearance,
    float weight
) {
    float cost = 0.0f;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const Vector2 start = route[index - 1];
        const Vector2 end = route[index];
        const float segmentLength = Vector2Distance(start, end);
        if (segmentLength <= 0.0001f) {
            continue;
        }

        const Vector2 midpoint = Vector2Scale(Vector2Add(start, end), 0.5f);
        float nearestDistance = std::numeric_limits<float>::infinity();
        for (const std::vector<Vector2>* markedPath : markedPaths) {
            if (markedPath) {
                nearestDistance = std::min(nearestDistance, distanceToPolyline(midpoint, *markedPath));
            }
        }

        if (nearestDistance < clearance) {
            const float overlap = (clearance - nearestDistance) / clearance;
            cost += segmentLength * overlap * weight;
        }
    }
    return cost;
}

float intersectionPenalty(const std::vector<Vector2>& route, const Intersection& intersection) {
    std::vector<const std::vector<Vector2>*> bikeLanes;
    std::vector<const Road*> roads;
    for (const Lane* lane : intersection.getConnectedLanes()) {
        const Road* road = intersection.getRoadForLane(lane);
        if (!road || std::find(roads.begin(), roads.end(), road) != roads.end()) {
            continue;
        }
        roads.push_back(road);
        bikeLanes.push_back(&road->getRightBikeLane());
        bikeLanes.push_back(&road->getLeftBikeLane());
    }

    const std::vector<std::vector<Vector2>> crossings = intersection.getPedestrianCrossings(0.0f);
    std::vector<const std::vector<Vector2>*> crosswalks;
    crosswalks.reserve(crossings.size());
    for (const auto& crossing : crossings) {
        crosswalks.push_back(&crossing);
    }

    return nearbyPathCost(route, bikeLanes, BIKE_LANE_CLEARANCE, 0.2f) +
        nearbyPathCost(route, crosswalks, CROSSWALK_CLEARANCE, 0.3f);
}
}

std::vector<Vector2> RoadGraph::buildTransitionSpline(
    Vector2 entry,
    Vector2 exit,
    Vector2 incomingDirection,
    Vector2 outgoingDirection,
    int samples,
    float handleRatio
) {
    const Vector2 incoming = normalizeOrZero(incomingDirection);
    const Vector2 outgoing = normalizeOrZero(outgoingDirection);
    const float handleLength = std::max(40.0f, Vector2Distance(entry, exit) * handleRatio);
    const Vector2 firstControl = Vector2Add(entry, Vector2Scale(incoming, handleLength));
    const Vector2 secondControl = Vector2Subtract(exit, Vector2Scale(outgoing, handleLength));

    std::vector<Vector2> points;
    points.reserve(static_cast<std::size_t>(samples) + 1);
    for (int index = 0; index <= samples; ++index) {
        const float t = static_cast<float>(index) / static_cast<float>(samples);
        const float oneMinusT = 1.0f - t;
        const float startWeight = oneMinusT * oneMinusT * oneMinusT;
        const float firstWeight = 3.0f * oneMinusT * oneMinusT * t;
        const float secondWeight = 3.0f * oneMinusT * t * t;
        const float endWeight = t * t * t;
        points.push_back({
            startWeight * entry.x + firstWeight * firstControl.x +
                secondWeight * secondControl.x + endWeight * exit.x,
            startWeight * entry.y + firstWeight * firstControl.y +
                secondWeight * secondControl.y + endWeight * exit.y
        });
    }
    return points;
}

void RoadGraph::addEdge(
    int startNode,
    int endNode,
    std::vector<Vector2> points,
    float extraCost
) {
    if (startNode < 0 || endNode < 0 || points.size() < 2) {
        return;
    }

    RoadGraphEdge edge;
    edge.startNode = startNode;
    edge.endNode = endNode;
    edge.cost = polylineLength(points) + std::max(0.0f, extraCost);
    edge.points = std::move(points);

    const int edgeIndex = static_cast<int>(edges.size());
    edges.push_back(std::move(edge));
    nodes[static_cast<std::size_t>(startNode)].outgoingEdges.push_back(edgeIndex);
}

void RoadGraph::addRoad(const Road& road, const std::vector<Intersection>& intersections) {
    auto addDirectionalLanes = [&](const std::vector<Lane>& lanes) {
        for (const Lane& lane : lanes) {
            const std::vector<Vector2>& points = lane.getWaypoints();
            if (points.size() < 2) {
                continue;
            }

            std::vector<int>& laneNodeIndices = laneNodes[&lane];
            laneNodeIndices.reserve(points.size());
            for (const Vector2 point : points) {
                laneNodeIndices.push_back(static_cast<int>(nodes.size()));
                nodes.push_back({ point, {} });
            }

            for (std::size_t index = 0; index + 1 < points.size(); ++index) {
                addEdge(
                    laneNodeIndices[index],
                    laneNodeIndices[index + 1],
                    { points[index], points[index + 1] }
                );
            }
        }

        // Lane changes connect only adjacent lanes traveling in the same direction.
        for (std::size_t laneIndex = 1; laneIndex < lanes.size(); ++laneIndex) {
            const Lane& firstLane = lanes[laneIndex - 1];
            const Lane& secondLane = lanes[laneIndex];
            const std::vector<Vector2>& firstPoints = firstLane.getWaypoints();
            const std::vector<Vector2>& secondPoints = secondLane.getWaypoints();
            const std::vector<int>& firstNodes = laneNodes[&firstLane];
            const std::vector<int>& secondNodes = laneNodes[&secondLane];
            const float availableLength = std::min(polylineLength(firstPoints), polylineLength(secondPoints));

            for (float centerDistance = LANE_CHANGE_LENGTH;
                 centerDistance + LANE_CHANGE_LENGTH * 0.5f < availableLength;
                 centerDistance += LANE_CHANGE_SPACING) {
                const float startDistance = centerDistance - LANE_CHANGE_LENGTH * 0.5f;
                const float endDistance = centerDistance + LANE_CHANGE_LENGTH * 0.5f;
                const std::size_t firstStart = waypointIndexAtDistance(firstPoints, startDistance);
                const std::size_t secondStart = waypointIndexAtDistance(secondPoints, startDistance);
                const std::size_t firstEnd = waypointIndexAtDistance(firstPoints, endDistance);
                const std::size_t secondEnd = waypointIndexAtDistance(secondPoints, endDistance);

                auto addLaneChange = [&](
                    const Lane& fromLane,
                    const std::vector<int>& fromNodes,
                    std::size_t fromIndex,
                    const Lane& toLane,
                    const std::vector<int>& toNodes,
                    std::size_t toIndex
                ) {
                    if (fromIndex >= fromNodes.size() || toIndex >= toNodes.size()) {
                        return;
                    }

                    std::vector<Vector2> spline = buildTransitionSpline(
                        nodes[static_cast<std::size_t>(fromNodes[fromIndex])].position,
                        nodes[static_cast<std::size_t>(toNodes[toIndex])].position,
                        fromLane.getDirectionAt(fromIndex),
                        toLane.getDirectionAt(toIndex),
                        LANE_CHANGE_SAMPLES
                    );
                    const bool entersIntersection = std::any_of(
                        intersections.begin(),
                        intersections.end(),
                        [&](const Intersection& intersection) {
                            return std::any_of(
                                spline.begin(),
                                spline.end(),
                                [&](Vector2 point) { return intersection.containsPoint(point); }
                            );
                        }
                    );
                    if (!entersIntersection) {
                        addEdge(fromNodes[fromIndex], toNodes[toIndex], std::move(spline));
                    }
                };

                addLaneChange(firstLane, firstNodes, firstStart, secondLane, secondNodes, secondEnd);
                addLaneChange(secondLane, secondNodes, secondStart, firstLane, firstNodes, firstEnd);
            }
        }
    };

    addDirectionalLanes(road.getRightLanes());
    addDirectionalLanes(road.getLeftLanes());
}

std::vector<RoadGraph::BoundaryNode> RoadGraph::findLaneBoundaryNodes(
    const std::vector<int>& laneNodeIndices,
    const Lane& lane,
    Vector2 intersectionCenter,
    Rectangle intersectionBounds,
    bool incoming
) const {
    const float boundaryDistance = std::max(intersectionBounds.width, intersectionBounds.height) * 0.5f + 20.0f;
    constexpr float MIN_APPROACH_DISTANCE = 40.0f;
    float bestProjection = incoming
        ? -std::numeric_limits<float>::infinity()
        : std::numeric_limits<float>::infinity();
    std::optional<BoundaryNode> best;
    auto distanceToBounds = [&](Vector2 point) {
        const float nearestX = std::clamp(point.x, intersectionBounds.x, intersectionBounds.x + intersectionBounds.width);
        const float nearestY = std::clamp(point.y, intersectionBounds.y, intersectionBounds.y + intersectionBounds.height);
        return Vector2Distance(point, { nearestX, nearestY });
    };

    for (std::size_t index = 0; index < laneNodeIndices.size(); ++index) {
        const Vector2 point = nodes[static_cast<std::size_t>(laneNodeIndices[index])].position;
        if (distanceToBounds(point) > TRANSITION_SEARCH_RADIUS) {
            continue;
        }

        const float projection = Vector2DotProduct(
            Vector2Subtract(point, intersectionCenter),
            lane.getDirectionAt(index)
        );
        const bool onApproachSide = incoming
            ? projection <= -boundaryDistance
            : projection >= boundaryDistance;
        const bool closer = incoming ? projection > bestProjection : projection < bestProjection;
        if (onApproachSide && closer) {
            best = BoundaryNode{ laneNodeIndices[index], index };
            bestProjection = projection;
        }
    }

    if (!best) {
        return {};
    }

    std::vector<BoundaryNode> candidates = { *best };
    const std::size_t towardIntersection = incoming
        ? best->waypointIndex + 1
        : (best->waypointIndex == 0 ? laneNodeIndices.size() : best->waypointIndex - 1);
    if (towardIntersection < laneNodeIndices.size()) {
        const Vector2 point = nodes[static_cast<std::size_t>(laneNodeIndices[towardIntersection])].position;
        const float projection = Vector2DotProduct(
            Vector2Subtract(point, intersectionCenter),
            lane.getDirectionAt(towardIntersection)
        );
        const bool remainsOnApproachSide = incoming
            ? projection <= -MIN_APPROACH_DISTANCE
            : projection >= MIN_APPROACH_DISTANCE;
        if (remainsOnApproachSide && distanceToBounds(point) <= TRANSITION_SEARCH_RADIUS) {
            candidates.push_back({ laneNodeIndices[towardIntersection], towardIntersection });
        }
    }
    return candidates;
}

void RoadGraph::addLaneTransition(
    const Lane* incomingLane,
    const Lane* outgoingLane,
    const Intersection& intersection
) {
    if (!incomingLane || !outgoingLane) {
        return;
    }

    const auto incoming = laneNodes.find(incomingLane);
    const auto outgoing = laneNodes.find(outgoingLane);
    if (incoming == laneNodes.end() || outgoing == laneNodes.end()) {
        return;
    }

    const Rectangle bounds = intersection.getBounds();
    const Vector2 center = {
        bounds.x + bounds.width * 0.5f,
        bounds.y + bounds.height * 0.5f
    };
    const std::vector<BoundaryNode> entries = findLaneBoundaryNodes(
        incoming->second,
        *incomingLane,
        center,
        bounds,
        true
    );
    const std::vector<BoundaryNode> exits = findLaneBoundaryNodes(
        outgoing->second,
        *outgoingLane,
        center,
        bounds,
        false
    );
    if (entries.empty() || exits.empty()) {
        return;
    }

    for (const BoundaryNode& entry : entries) {
        for (const BoundaryNode& exit : exits) {
            std::vector<Vector2> spline = buildTransitionSpline(
                nodes[static_cast<std::size_t>(entry.nodeIndex)].position,
                nodes[static_cast<std::size_t>(exit.nodeIndex)].position,
                incomingLane->getDirectionAt(entry.waypointIndex),
                outgoingLane->getDirectionAt(exit.waypointIndex),
                18,
                TURN_HANDLE_RATIO
            );
            const float penalty = std::min(
                intersectionPenalty(spline, intersection),
                polylineLength(spline) * MAX_INTERSECTION_PENALTY_RATIO
            );
            addEdge(entry.nodeIndex, exit.nodeIndex, std::move(spline), penalty);
        }
    }
}

std::optional<int> RoadGraph::findNearestNode(Vector2 point, float maxDistance) const {
    std::optional<int> nearest;
    float nearestDistance = maxDistance;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const float distance = Vector2Distance(nodes[index].position, point);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = static_cast<int>(index);
        }
    }
    return nearest;
}

std::optional<Vector2> RoadGraph::snapPointToRoad(Vector2 worldPoint, float maxDistance) const {
    const auto node = findNearestNode(worldPoint, maxDistance);
    return node ? std::optional<Vector2>(nodes[static_cast<std::size_t>(*node)].position) : std::nullopt;
}

std::vector<Vector2> RoadGraph::findRoute(Vector2 start, Vector2 end) const {
    const auto startNode = findNearestNode(start, 200.0f);
    const auto endNode = findNearestNode(end, 200.0f);
    if (!startNode || !endNode) {
        return {};
    }
    if (*startNode == *endNode) {
        return { nodes[static_cast<std::size_t>(*startNode)].position };
    }

    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<float> distance(nodes.size(), infinity);
    std::vector<int> previousNode(nodes.size(), -1);
    std::vector<int> previousEdge(nodes.size(), -1);
    using QueueEntry = std::pair<float, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;

    distance[static_cast<std::size_t>(*startNode)] = 0.0f;
    frontier.push({ 0.0f, *startNode });
    while (!frontier.empty()) {
        const auto [currentDistance, currentNode] = frontier.top();
        frontier.pop();
        if (currentDistance != distance[static_cast<std::size_t>(currentNode)]) {
            continue;
        }
        if (currentNode == *endNode) {
            break;
        }

        for (const int edgeIndex : nodes[static_cast<std::size_t>(currentNode)].outgoingEdges) {
            const RoadGraphEdge& edge = edges[static_cast<std::size_t>(edgeIndex)];
            const float nextDistance = currentDistance + edge.cost;
            const std::size_t nextNode = static_cast<std::size_t>(edge.endNode);
            if (nextDistance < distance[nextNode]) {
                distance[nextNode] = nextDistance;
                previousNode[nextNode] = currentNode;
                previousEdge[nextNode] = edgeIndex;
                frontier.push({ nextDistance, edge.endNode });
            }
        }
    }

    if (!std::isfinite(distance[static_cast<std::size_t>(*endNode)])) {
        return {};
    }

    std::vector<int> pathEdges;
    int currentNode = *endNode;
    while (currentNode != *startNode) {
        const int edgeIndex = previousEdge[static_cast<std::size_t>(currentNode)];
        if (edgeIndex < 0) {
            return {};
        }
        pathEdges.push_back(edgeIndex);
        currentNode = previousNode[static_cast<std::size_t>(currentNode)];
    }
    std::reverse(pathEdges.begin(), pathEdges.end());

    std::vector<Vector2> route = { nodes[static_cast<std::size_t>(*startNode)].position };
    for (const int edgeIndex : pathEdges) {
        const std::vector<Vector2>& points = edges[static_cast<std::size_t>(edgeIndex)].points;
        route.insert(route.end(), points.begin() + 1, points.end());
    }
    return route;
}
