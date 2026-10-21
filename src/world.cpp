#include "world.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr float INTERSECTION_DRAW_SIZE = 220.0f;
constexpr float INTERSECTION_DEDUPE_DISTANCE = 30.0f;

float cross(Vector2 first, Vector2 second) {
    return first.x * second.y - first.y * second.x;
}

Vector2 directionAt(const Road& road, Vector2 point) {
    const std::vector<Vector2>& centerline = road.getCenterline();
    Vector2 closestDirection = { 1.0f, 0.0f };
    float closestDistanceSquared = std::numeric_limits<float>::infinity();

    for (std::size_t index = 1; index < centerline.size(); ++index) {
        const Vector2 start = centerline[index - 1];
        const Vector2 segment = Vector2Subtract(centerline[index], start);
        const float segmentLengthSquared = Vector2LengthSqr(segment);
        if (segmentLengthSquared <= 0.0001f) {
            continue;
        }

        const float projection = std::clamp(
            Vector2DotProduct(Vector2Subtract(point, start), segment) / segmentLengthSquared,
            0.0f,
            1.0f
        );
        const Vector2 closestPoint = Vector2Add(start, Vector2Scale(segment, projection));
        const float distanceSquared = Vector2LengthSqr(Vector2Subtract(point, closestPoint));
        if (distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = distanceSquared;
            closestDirection = Vector2Scale(segment, 1.0f / std::sqrt(segmentLengthSquared));
        }
    }

    return closestDirection;
}

std::optional<Vector2> intersectSegments(Vector2 startA, Vector2 endA, Vector2 startB, Vector2 endB) {
    const Vector2 directionA = { endA.x - startA.x, endA.y - startA.y };
    const Vector2 directionB = { endB.x - startB.x, endB.y - startB.y };
    const Vector2 betweenStarts = { startB.x - startA.x, startB.y - startA.y };
    const float denominator = cross(directionA, directionB);

    if (std::abs(denominator) <= 0.0001f) {
        return std::nullopt;
    }

    const float amountA = cross(betweenStarts, directionB) / denominator;
    const float amountB = cross(betweenStarts, directionA) / denominator;
    if (amountA < -0.0001f || amountA > 1.0001f || amountB < -0.0001f || amountB > 1.0001f) {
        return std::nullopt;
    }

    return Vector2{
        startA.x + directionA.x * amountA,
        startA.y + directionA.y * amountA
    };
}

std::vector<Vector2> makeRoadEdge(const Road& road, float offset) {
    const std::vector<Vector2>& centerline = road.getCenterline();
    std::vector<Vector2> edge;
    edge.reserve(centerline.size());

    for (std::size_t index = 0; index < centerline.size(); ++index) {
        const std::size_t previous = index == 0 ? index : index - 1;
        const std::size_t next = index + 1 < centerline.size() ? index + 1 : index;
        Vector2 direction = Vector2Subtract(centerline[next], centerline[previous]);
        const float length = Vector2Length(direction);
        if (length <= 0.0001f) {
            edge.push_back(centerline[index]);
            continue;
        }

        direction = Vector2Scale(direction, 1.0f / length);
        const Vector2 normal = { -direction.y, direction.x };
        edge.push_back(Vector2Add(centerline[index], Vector2Scale(normal, offset)));
    }

    return edge;
}

std::optional<Vector2> findNearestBoundaryCrossing(
    const std::vector<Vector2>& firstEdge,
    const std::vector<Vector2>& secondEdge,
    Vector2 center
) {
    std::optional<Vector2> closest;
    float closestDistanceSquared = std::numeric_limits<float>::infinity();

    for (std::size_t firstIndex = 1; firstIndex < firstEdge.size(); ++firstIndex) {
        for (std::size_t secondIndex = 1; secondIndex < secondEdge.size(); ++secondIndex) {
            const auto point = intersectSegments(
                firstEdge[firstIndex - 1],
                firstEdge[firstIndex],
                secondEdge[secondIndex - 1],
                secondEdge[secondIndex]
            );
            if (!point) {
                continue;
            }

            const float distanceSquared = Vector2LengthSqr(Vector2Subtract(center, *point));
            if (distanceSquared < closestDistanceSquared) {
                closest = point;
                closestDistanceSquared = distanceSquared;
            }
        }
    }

    return closest;
}

std::vector<Vector2> makeTangentIntersectionSurface(Vector2 center, const Road& first, const Road& second) {
    const Vector2 firstDirection = directionAt(first, center);
    const Vector2 secondDirection = directionAt(second, center);
    const Vector2 firstNormal = { -firstDirection.y, firstDirection.x };
    const Vector2 secondNormal = { -secondDirection.y, secondDirection.x };
    const float determinant = cross(firstNormal, secondNormal);
    if (std::abs(determinant) <= 0.0001f) {
        return {};
    }

    auto corner = [&](float firstOffset, float secondOffset) {
        const Vector2 offset = {
            (firstOffset * secondNormal.y - firstNormal.y * secondOffset) / determinant,
            (firstNormal.x * secondOffset - firstOffset * secondNormal.x) / determinant
        };
        return Vector2Add(center, offset);
    };

    const float firstHalfWidth = first.getWidth() * 0.5f;
    const float secondHalfWidth = second.getWidth() * 0.5f;
    return {
        corner(-firstHalfWidth, -secondHalfWidth),
        corner(firstHalfWidth, -secondHalfWidth),
        corner(firstHalfWidth, secondHalfWidth),
        corner(-firstHalfWidth, secondHalfWidth)
    };
}

std::vector<Vector2> makeIntersectionSurface(Vector2 center, const Road& first, const Road& second) {
    const float firstHalfWidth = first.getWidth() * 0.5f;
    const float secondHalfWidth = second.getWidth() * 0.5f;
    const std::vector<Vector2> firstNegativeEdge = makeRoadEdge(first, -firstHalfWidth);
    const std::vector<Vector2> firstPositiveEdge = makeRoadEdge(first, firstHalfWidth);
    const std::vector<Vector2> secondNegativeEdge = makeRoadEdge(second, -secondHalfWidth);
    const std::vector<Vector2> secondPositiveEdge = makeRoadEdge(second, secondHalfWidth);

    const auto negativeNegative = findNearestBoundaryCrossing(firstNegativeEdge, secondNegativeEdge, center);
    const auto positiveNegative = findNearestBoundaryCrossing(firstPositiveEdge, secondNegativeEdge, center);
    const auto positivePositive = findNearestBoundaryCrossing(firstPositiveEdge, secondPositiveEdge, center);
    const auto negativePositive = findNearestBoundaryCrossing(firstNegativeEdge, secondPositiveEdge, center);

    if (!negativeNegative || !positiveNegative || !positivePositive || !negativePositive) {
        return makeTangentIntersectionSurface(center, first, second);
    }

    return { *negativeNegative, *positiveNegative, *positivePositive, *negativePositive };
}

std::vector<Vector2> findCenterlineIntersections(const Road& first, const Road& second) {
    const std::vector<Vector2>& firstLine = first.getCenterline();
    const std::vector<Vector2>& secondLine = second.getCenterline();
    std::vector<Vector2> intersections;

    for (std::size_t firstIndex = 1; firstIndex < firstLine.size(); ++firstIndex) {
        for (std::size_t secondIndex = 1; secondIndex < secondLine.size(); ++secondIndex) {
            const auto point = intersectSegments(
                firstLine[firstIndex - 1],
                firstLine[firstIndex],
                secondLine[secondIndex - 1],
                secondLine[secondIndex]
            );
            if (!point) {
                continue;
            }

            const bool alreadyAdded = std::any_of(
                intersections.begin(),
                intersections.end(),
                [&](Vector2 existing) {
                    return Vector2Distance(existing, *point) < INTERSECTION_DEDUPE_DISTANCE;
                }
            );
            if (!alreadyAdded) {
                intersections.push_back(*point);
            }
        }
    }

    return intersections;
}

}

World World::makeDowntownGrid() {
    constexpr int BLOCKS_PER_SIDE = 5;
    constexpr float BLOCK_SPACING = 500.0f;
    constexpr float ROAD_WIDTH = 160.0f;
    constexpr float ROAD_SAMPLE_SPACING = 20.0f;
    constexpr float CURVE_AMPLITUDE = 75.0f;

    World world;
    const float firstGridCoordinate = -BLOCK_SPACING * BLOCKS_PER_SIDE / 2.0f;
    const float mapStart = firstGridCoordinate - BLOCK_SPACING / 2.0f;
    const float mapEnd = -mapStart;
    std::vector<float> gridCoordinates;
    gridCoordinates.reserve(BLOCKS_PER_SIDE + 1);
    for (int index = 0; index <= BLOCKS_PER_SIDE; ++index) {
        gridCoordinates.push_back(firstGridCoordinate + index * BLOCK_SPACING);
    }

    auto addHorizontalStreet = [&](int row) {
        const float baseY = gridCoordinates[static_cast<std::size_t>(row)];
        const float amplitude = row == 1 ? CURVE_AMPLITUDE : (row == 4 ? 45.0f : 0.0f);
        const float frequency = row == 4
            ? 2.0f * PI / BLOCK_SPACING
            : PI / BLOCK_SPACING;
        std::vector<Vector2> centerline;

        for (float x = mapStart; x < mapEnd; x += ROAD_SAMPLE_SPACING) {
            const float phase = (x - gridCoordinates.front()) * frequency;
            centerline.push_back({ x, baseY + amplitude * std::sin(phase) });
        }
        const float endPhase = (mapEnd - gridCoordinates.front()) * frequency;
        centerline.push_back({ mapEnd, baseY + amplitude * std::sin(endPhase) });
        world.addRoad(Road(std::move(centerline), ROAD_WIDTH));
    };

    auto addVerticalStreet = [&](int column) {
        const float baseX = gridCoordinates[static_cast<std::size_t>(column)];
        const float amplitude = column == 1 ? 55.0f : (column == 4 ? CURVE_AMPLITUDE : 0.0f);
        const float frequency = column == 1
            ? 2.0f * PI / BLOCK_SPACING
            : PI / BLOCK_SPACING;
        std::vector<Vector2> centerline;

        for (float y = mapStart; y < mapEnd; y += ROAD_SAMPLE_SPACING) {
            const float phase = (y - gridCoordinates.front()) * frequency;
            centerline.push_back({ baseX + amplitude * std::sin(phase), y });
        }
        const float endPhase = (mapEnd - gridCoordinates.front()) * frequency;
        centerline.push_back({ baseX + amplitude * std::sin(endPhase), mapEnd });
        world.addRoad(Road(std::move(centerline), ROAD_WIDTH));
    };

    for (int row = 0; row <= BLOCKS_PER_SIDE; ++row) {
        addHorizontalStreet(row);
    }
    for (int column = 0; column <= BLOCKS_PER_SIDE; ++column) {
        addVerticalStreet(column);
    }

    world.computeIntersections();
    return world;
}

std::optional<Vector2> World::findNearestRoadPoint(Vector2 worldPoint, float maxDistance) const {
    return graph.snapPointToRoad(worldPoint, maxDistance);
}

std::vector<Vector2> World::buildRouteBetweenPoints(Vector2 start, Vector2 end) const {
    const auto snappedStart = graph.snapPointToRoad(start, 200.0f);
    const auto snappedEnd = graph.snapPointToRoad(end, 200.0f);
    if (!snappedStart || !snappedEnd) {
        return {};
    }

    return graph.findRoute(start, end);
}

void World::addRoad(Road road) {
    roads.push_back(std::move(road));
}

const std::vector<Road>& World::getRoads() const {
    return roads;
}

void World::computeIntersections() {
    intersections.clear();

    for (std::size_t firstRoad = 0; firstRoad < roads.size(); ++firstRoad) {
        for (std::size_t secondRoad = firstRoad + 1; secondRoad < roads.size(); ++secondRoad) {
            for (const Vector2 center : findCenterlineIntersections(roads[firstRoad], roads[secondRoad])) {
                const float halfSize = INTERSECTION_DRAW_SIZE * 0.5f;
                auto surface = makeIntersectionSurface(center, roads[firstRoad], roads[secondRoad]);
                Intersection intersection({
                    center.x - halfSize,
                    center.y - halfSize,
                    INTERSECTION_DRAW_SIZE,
                    INTERSECTION_DRAW_SIZE
                }, std::move(surface));
                intersection.addLane(&roads[firstRoad].getRightLane());
                intersection.addLane(&roads[firstRoad].getLeftLane());
                intersection.addLane(&roads[secondRoad].getRightLane());
                intersection.addLane(&roads[secondRoad].getLeftLane());
                intersection.computeTransitions();
                intersections.push_back(std::move(intersection));
            }
        }
    }

    rebuildGraph();
}

void World::rebuildGraph() {
    graph = RoadGraph();

    for (const Road& road : roads) {
        graph.addRoad(road);
    }

    for (const Intersection& intersection : intersections) {
        for (Lane* incomingLane : intersection.getConnectedLanes()) {
            for (Lane* outgoingLane : intersection.getNextLanes(incomingLane)) {
                graph.addLaneTransition(incomingLane, outgoingLane, intersection.getBounds());
            }
        }
    }
}

void World::Draw() const {
    for (const Road& road : roads) {
        road.Draw(intersections);
    }

    for (const Intersection& intersection : intersections) {
        intersection.Draw();
    }
}
