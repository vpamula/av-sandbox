#include "world.h"

#include <algorithm>
#include <utility>

std::optional<Vector2> World::findNearestRoadPoint(Vector2 worldPoint, float maxDistance) const {
    return graph.snapPointToRoad(worldPoint, maxDistance);
}

std::vector<Vector2> World::buildRouteBetweenPoints(Vector2 start, Vector2 end) const {
    const auto snappedStart = graph.snapPointToRoad(start, 200.0f);
    const auto snappedEnd = graph.snapPointToRoad(end, 200.0f);
    if (!snappedStart || !snappedEnd) {
        return {};
    }

    return graph.findRoute(*snappedStart, *snappedEnd);
}

void World::addRoad(Road road) {
    roads.push_back(std::move(road));
}

const std::vector<Road>& World::getRoads() const {
    return roads;
}

void World::computeIntersections() {
    intersections.clear();

    for (size_t firstRoad = 0; firstRoad < roads.size(); ++firstRoad) {
        for (size_t secondRoad = firstRoad + 1; secondRoad < roads.size(); ++secondRoad) {
            const Rectangle firstBounds = roads[firstRoad].getBounds();
            const Rectangle secondBounds = roads[secondRoad].getBounds();
            if (!CheckCollisionRecs(firstBounds, secondBounds)) {
                continue;
            }

            const float left = std::max(firstBounds.x, secondBounds.x);
            const float right = std::min(firstBounds.x + firstBounds.width, secondBounds.x + secondBounds.width);
            const float top = std::max(firstBounds.y, secondBounds.y);
            const float bottom = std::min(firstBounds.y + firstBounds.height, secondBounds.y + secondBounds.height);

            Intersection intersection({ left, top, right - left, bottom - top });
            intersection.addLane(&roads[firstRoad].getRightLane());
            intersection.addLane(&roads[firstRoad].getLeftLane());
            intersection.addLane(&roads[secondRoad].getRightLane());
            intersection.addLane(&roads[secondRoad].getLeftLane());
            intersection.computeTransitions();
            intersections.push_back(std::move(intersection));
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
