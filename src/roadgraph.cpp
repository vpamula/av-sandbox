#include "roadgraph.h"

#include "intersection.h"
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
constexpr float LANE_CHANGE_SPACING = 500.0f;
constexpr float LANE_CHANGE_LENGTH = 160.0f;
constexpr int LANE_CHANGE_SAMPLES = 18;
constexpr float BIKE_LANE_CLEARANCE = 36.0f;
constexpr float BIKE_LANE_OVERLAP_WEIGHT = 6.0f;
constexpr float CROSSWALK_CLEARANCE = 48.0f;
constexpr float CROSSWALK_OVERLAP_WEIGHT = 8.0f;
constexpr float TURN_HANDLE_RATIO = 0.58f;

Vector2 normalizeOrZero(Vector2 value) {
    const float lengthSquared = value.x * value.x + value.y * value.y;
    if (lengthSquared <= 0.0001f) {
        return { 0.0f, 0.0f };
    }
    return Vector2Scale(value, 1.0f / std::sqrt(lengthSquared));
}

float polylineLength(const std::vector<Vector2>& points) {
    float length = 0.0f;
    for (std::size_t index = 1; index < points.size(); ++index) {
        length += Vector2Distance(points[index - 1], points[index]);
    }
    return length;
}

struct ClosestPolylineSegment {
    float distance = std::numeric_limits<float>::infinity();
    Vector2 direction = { 0.0f, 0.0f };
};

ClosestPolylineSegment closestPolylineSegment(
    Vector2 point,
    const std::vector<Vector2>& polyline
) {
    ClosestPolylineSegment closest;
    for (std::size_t index = 1; index < polyline.size(); ++index) {
        const Vector2 start = polyline[index - 1];
        const Vector2 segment = Vector2Subtract(polyline[index], start);
        const float segmentLengthSquared = Vector2LengthSqr(segment);
        if (segmentLengthSquared <= 0.0001f) {
            continue;
        }

        const float projection = std::clamp(
            Vector2DotProduct(Vector2Subtract(point, start), segment) / segmentLengthSquared,
            0.0f,
            1.0f
        );
        const Vector2 nearest = Vector2Add(start, Vector2Scale(segment, projection));
        const float distance = Vector2Distance(point, nearest);
        if (distance < closest.distance) {
            closest.distance = distance;
            closest.direction = normalizeOrZero(segment);
        }
    }
    return closest;
}

float overlapRoutingPenalty(
    const std::vector<Vector2>& route,
    const std::vector<const std::vector<Vector2>*>& obstacles,
    float clearance,
    float weight,
    float crossingWeight
) {
    float penalty = 0.0f;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const Vector2 segment = Vector2Subtract(route[index], route[index - 1]);
        const float segmentLength = Vector2Length(segment);
        if (segmentLength <= 0.0001f) {
            continue;
        }

        const Vector2 midpoint = Vector2Scale(Vector2Add(route[index - 1], route[index]), 0.5f);
        const Vector2 routeDirection = Vector2Scale(segment, 1.0f / segmentLength);
        for (const std::vector<Vector2>* obstacle : obstacles) {
            if (!obstacle) {
                continue;
            }
            const ClosestPolylineSegment closest = closestPolylineSegment(midpoint, *obstacle);
            if (closest.distance >= clearance) {
                continue;
            }

            // Give unavoidable crossings a small cost and strongly discourage
            // driving along the marked corridor. This also lets straight road
            // segments prefer a path with fewer or shorter crosswalk overlaps.
            const float alignment = std::abs(
                Vector2DotProduct(routeDirection, closest.direction)
            );
            const float parallelism = std::clamp((alignment - 0.35f) / 0.65f, 0.0f, 1.0f);
            const float alignmentCost = crossingWeight +
                (1.0f - crossingWeight) * parallelism;
            const float proximity = (clearance - closest.distance) / clearance;
            penalty += segmentLength * proximity * alignmentCost * weight;
        }
    }
    return penalty;
}

float intersectionOverlapPenalty(
    const std::vector<Vector2>& routeSegment,
    const Intersection& intersection
) {
    if (routeSegment.size() < 2) {
        return 0.0f;
    }

    const Vector2 midpoint = Vector2Scale(
        Vector2Add(routeSegment.front(), routeSegment.back()),
        0.5f
    );
    if (!intersection.containsPointWithinDistance(midpoint, 40.0f)) {
        return 0.0f;
    }

    std::vector<const std::vector<Vector2>*> bikeLanePaths;
    std::vector<const Road*> connectedRoads;
    for (const Lane* lane : intersection.getConnectedLanes()) {
        const Road* road = intersection.getRoadForLane(lane);
        if (!road || std::find(connectedRoads.begin(), connectedRoads.end(), road) != connectedRoads.end()) {
            continue;
        }
        connectedRoads.push_back(road);
        bikeLanePaths.push_back(&road->getRightBikeLane());
        bikeLanePaths.push_back(&road->getLeftBikeLane());
    }

    const std::vector<std::vector<Vector2>> crossings = intersection.getPedestrianCrossings(0.0f);
    std::vector<const std::vector<Vector2>*> crossingPaths;
    crossingPaths.reserve(crossings.size());
    for (const std::vector<Vector2>& crossing : crossings) {
        crossingPaths.push_back(&crossing);
    }

    return overlapRoutingPenalty(
        routeSegment,
        bikeLanePaths,
        BIKE_LANE_CLEARANCE,
        BIKE_LANE_OVERLAP_WEIGHT,
        0.18f
    ) + overlapRoutingPenalty(
        routeSegment,
        crossingPaths,
        CROSSWALK_CLEARANCE,
        CROSSWALK_OVERLAP_WEIGHT,
        0.12f
    );
}

std::size_t waypointIndexAtDistance(const std::vector<Vector2>& points, float targetDistance) {
    if (points.size() < 2 || targetDistance <= 0.0f) {
        return 0;
    }

    std::size_t bestIndex = 0;
    float bestDifference = std::abs(targetDistance);
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
}

std::vector<Vector2> RoadGraph::buildTransitionSpline(
    Vector2 entryPoint,
    Vector2 exitPoint,
    Vector2 incomingDirection,
    Vector2 outgoingDirection,
    int samples,
    float handleRatio
) {
    const Vector2 incomingDir = normalizeOrZero(incomingDirection);
    const Vector2 outgoingDir = normalizeOrZero(outgoingDirection);
    const float travelLength = std::max(
        40.0f,
        Vector2Distance(entryPoint, exitPoint) * handleRatio
    );
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

void RoadGraph::addRoad(const Road& road, const std::vector<Intersection>& intersections) {
    auto addDirectionalLanes = [&](const std::vector<Lane>& lanes) {
        for (const Lane& laneValue : lanes) {
            const Lane* lane = &laneValue;
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
                for (const Intersection& intersection : intersections) {
                    edge.routingPenalty += intersectionOverlapPenalty(edge.points, intersection);
                }

                const int edgeIndex = static_cast<int>(edges.size());
                edges.push_back(std::move(edge));
                nodes[static_cast<std::size_t>(laneNodeIndices[index])].outgoingEdges.push_back(edgeIndex);
            }
        }

        for (std::size_t laneIndex = 1; laneIndex < lanes.size(); ++laneIndex) {
            const Lane& firstLane = lanes[laneIndex - 1];
            const Lane& secondLane = lanes[laneIndex];
            const std::vector<int>& firstNodes = laneNodes[&firstLane];
            const std::vector<int>& secondNodes = laneNodes[&secondLane];
            const float availableLength = std::min(
                polylineLength(firstLane.getWaypoints()),
                polylineLength(secondLane.getWaypoints())
            );

            for (float centerDistance = LANE_CHANGE_SPACING;
                 centerDistance < availableLength;
                 centerDistance += LANE_CHANGE_SPACING) {
                const float startDistance = centerDistance - LANE_CHANGE_LENGTH * 0.5f;
                const float endDistance = centerDistance + LANE_CHANGE_LENGTH * 0.5f;
                if (startDistance < 0.0f || endDistance >= availableLength) {
                    continue;
                }

                const std::size_t firstStartIndex = waypointIndexAtDistance(
                    firstLane.getWaypoints(), startDistance
                );
                const std::size_t secondStartIndex = waypointIndexAtDistance(
                    secondLane.getWaypoints(), startDistance
                );
                const std::size_t firstEndIndex = waypointIndexAtDistance(
                    firstLane.getWaypoints(), endDistance
                );
                const std::size_t secondEndIndex = waypointIndexAtDistance(
                    secondLane.getWaypoints(), endDistance
                );
                if (firstStartIndex >= firstNodes.size() || firstEndIndex >= firstNodes.size() ||
                    secondStartIndex >= secondNodes.size() || secondEndIndex >= secondNodes.size()) {
                    continue;
                }

                auto addLaneChange = [&](
                    const Lane& fromLane,
                    std::size_t fromWaypoint,
                    const std::vector<int>& fromNodes,
                    const Lane& toLane,
                    std::size_t toWaypoint,
                    const std::vector<int>& toNodes
                ) {
                    RoadGraphEdge edge;
                    edge.startNode = fromNodes[fromWaypoint];
                    edge.endNode = toNodes[toWaypoint];
                    edge.points = buildTransitionSpline(
                        nodes[static_cast<std::size_t>(edge.startNode)].position,
                        nodes[static_cast<std::size_t>(edge.endNode)].position,
                        fromLane.getDirectionAt(fromWaypoint),
                        toLane.getDirectionAt(toWaypoint),
                        LANE_CHANGE_SAMPLES
                    );
                    const bool crossesIntersection = std::any_of(
                        intersections.begin(),
                        intersections.end(),
                        [&](const Intersection& intersection) {
                            return std::any_of(
                                edge.points.begin(),
                                edge.points.end(),
                                [&](Vector2 point) { return intersection.containsPoint(point); }
                            );
                        }
                    );
                    if (crossesIntersection) {
                        return;
                    }

                    const int edgeIndex = static_cast<int>(edges.size());
                    edges.push_back(std::move(edge));
                    nodes[static_cast<std::size_t>(edges.back().startNode)].outgoingEdges.push_back(edgeIndex);
                };

                addLaneChange(
                    firstLane,
                    firstStartIndex,
                    firstNodes,
                    secondLane,
                    secondEndIndex,
                    secondNodes
                );
                addLaneChange(
                    secondLane,
                    secondStartIndex,
                    secondNodes,
                    firstLane,
                    firstEndIndex,
                    firstNodes
                );
            }
        }
    };

    addDirectionalLanes(road.getRightLanes());
    addDirectionalLanes(road.getLeftLanes());
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
    const Intersection& intersection
) {
    if (!incomingLane || !outgoingLane) {
        return;
    }

    const auto incomingIt = laneNodes.find(incomingLane);
    const auto outgoingIt = laneNodes.find(outgoingLane);
    if (incomingIt == laneNodes.end() || outgoingIt == laneNodes.end()) {
        return;
    }

    const Rectangle junctionBounds = intersection.getBounds();
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
        outgoingLane->getDirectionAt(outgoingBoundary->waypointIndex),
        18,
        TURN_HANDLE_RATIO
    );

    std::vector<const std::vector<Vector2>*> bikeLanePaths;
    std::vector<const std::vector<Vector2>*> crossingPaths;
    std::vector<const Road*> connectedRoads;
    for (const Lane* lane : intersection.getConnectedLanes()) {
        const Road* road = intersection.getRoadForLane(lane);
        if (!road || std::find(connectedRoads.begin(), connectedRoads.end(), road) != connectedRoads.end()) {
            continue;
        }
        connectedRoads.push_back(road);
        bikeLanePaths.push_back(&road->getRightBikeLane());
        bikeLanePaths.push_back(&road->getLeftBikeLane());
    }

    const std::vector<std::vector<Vector2>> crossings = intersection.getPedestrianCrossings(0.0f);
    crossingPaths.reserve(crossings.size());
    for (const std::vector<Vector2>& crossing : crossings) {
        crossingPaths.push_back(&crossing);
    }
    edge.routingPenalty = overlapRoutingPenalty(
        edge.points,
        bikeLanePaths,
        BIKE_LANE_CLEARANCE,
        BIKE_LANE_OVERLAP_WEIGHT,
        0.18f
    ) + overlapRoutingPenalty(
        edge.points,
        crossingPaths,
        CROSSWALK_CLEARANCE,
        CROSSWALK_OVERLAP_WEIGHT,
        0.12f
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
            const float nextCost = currentCost + edgeLength + edge.routingPenalty;
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
