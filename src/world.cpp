#include "world.h"

void World::addRoad(Road road) {
    roads.push_back(road);
}

std::vector<Road>& World::getRoads() {
    return roads;
}

std::vector<Intersection>& World::getIntersections() {
    return intersections;
}

void World::computeIntersections() {
    intersections.clear();

    for (int i = 0; i < roads.size(); i++) {
        for (int j = i + 1; j < roads.size(); j++) {
            Rectangle a = roads[i].getBounds();
            Rectangle b = roads[j].getBounds();

            if (CheckCollisionRecs(a, b)) {

                float left = std::max(
                    a.x,
                    b.x
                );

                float right = std::min(
                        a.x + a.width,
                        b.x + b.width
                    );

                float top = std::max(
                        a.y,
                        b.y
                    );

                float bottom = std::min(
                        a.y + a.height,
                        b.y + b.height
                    );

                Rectangle overlap = {
                    left,
                    top,
                    right - left,
                    bottom - top
                };

                intersections.push_back(
                    Intersection(overlap)
                );
            }
        }
    }
}

void World::Draw() {
    for (Road& road : roads) {
        road.Draw(intersections);
    }

    for (Intersection& intersection : intersections) {
        intersection.Draw();
    }
}