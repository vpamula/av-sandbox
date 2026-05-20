#include "lane.h"
#include "raymath.h"
#include <cmath>

Lane::Lane(float center_x, float center_y, float heading, float width, float length) {
    this->center_x = center_x;
    this->center_y = center_y;
    this->heading = heading;
    this->width = width;
    this->length = length;

}

std::vector<Vector2> Lane::generateWaypoints() {
    std::vector<Vector2> waypoints;
    float spacing = 100.0f;
    Vector2 direction = {cos(heading), sin(heading)};
    for (float s = -length / 2; s <= length / 2; s += spacing) {
        Vector2 point = {center_x + direction.x * s, center_y + direction.y * s};
        waypoints.push_back(point);
    }
    return waypoints;
}

Vector2 Lane::getDirection() {
    return (Vector2) {cos(heading), sin(heading)};
}

std::vector<Vector2>

Lane::generateWaypointsBetween(Vector2 startPoint, Vector2 endPoint) {
    std::vector<Vector2> waypoints;
    float spacing = 100.0f;
    Vector2 direction = getDirection();
    Vector2 laneStart = getStartPoint();
    float startProjection = (startPoint.x - laneStart.x) * direction.x + (startPoint.y - laneStart.y) * direction.y;
    float endProjection = (endPoint.x - laneStart.x) * direction.x + (endPoint.y - laneStart.y) * direction.y;
    for (float s = startProjection; s <= endProjection; s += spacing) {
        Vector2 point = {laneStart.x + direction.x * s, laneStart.y + direction.y * s};
        waypoints.push_back(point);
    }
    return waypoints;
}