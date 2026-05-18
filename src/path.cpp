#include "path.h"

void Path::addWaypoint(Vector2 point) {
    waypoints.push_back(point);
}

std::vector<Vector2>& Path::getWaypoints() {
    return waypoints;
}

void Path::Draw() {
    for (Vector2 wp : waypoints) {
        DrawCircleV(wp, 6, BLUE);
    }
}