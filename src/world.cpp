#include "world.h"
#include <iostream>

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

                Intersection intersection(overlap);

                intersection.addLane(&roads[i].getRightLane());
                intersection.addLane(&roads[i].getLeftLane());
                intersection.addLane(&roads[j].getRightLane());
                intersection.addLane(&roads[j].getLeftLane());
                intersection.computeTransitions();
                intersections.push_back(intersection);
                std::cout << "Intersection created with " << intersection.getConnectedLanes().size() << " lanes\n";
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