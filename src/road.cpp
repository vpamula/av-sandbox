#include "road.h"
#include <cmath>

Road::Road(float center_x, float center_y, float heading, float width, float length) 
: rightLane(center_x + cos(heading + PI/2) * width / 4, center_y + sin(heading + PI/2) * width / 4, heading, width / 2, length),
leftLane(center_x, center_y - width / 4, heading + PI, width / 2, length) {
    this->center_x = center_x;
    this->center_y = center_y;
    this->heading = heading;
    this->width = width;
    this->length = length;
}

void Road::Draw() {
    Rectangle rect = {center_x, center_y, length, width};
    Vector2 origin = {length / 2, width / 2};
    DrawRectanglePro(rect, origin, heading * RAD2DEG, DARKGRAY);

    float dashLength = 80.0f;
    float dashGap = 60.0f;
    float dashWidth = 8.0f;

    float dirX = cos(heading);
    float dirY = sin(heading);
    for (float s = -length / 2; s < length / 2; s += dashLength + dashGap) {
        float x = center_x + s * dirX;
        float y = center_y + s * dirY;
        Rectangle dash = {x, y, dashLength, dashWidth};
        Vector2 dashOrigin = {dashLength / 2, dashWidth / 2};
        DrawRectanglePro(dash, dashOrigin, heading * RAD2DEG, YELLOW);
    }

}

std::vector<Vector2> Road::generateLaneWaypoints(bool rightLane) {
    std::vector<Vector2> laneWaypoints;
    float dirX = cos(heading);
    float dirY = sin(heading);
    float perpX = -sin(heading);
    float perpY = cos(heading);

    float laneOffset;

    if (rightLane) {
        laneOffset = width / 4;
    } else {
        laneOffset = -width / 4;
    }

    float waypointSpacing = 100.0f;

    for (float s = -length / 2; s < length / 2; s += waypointSpacing) {
        float x = center_x + s * dirX + laneOffset * perpX;
        float y = center_y + s * dirY + laneOffset * perpY;
        laneWaypoints.push_back((Vector2){x, y});
    }

    return laneWaypoints;

}

Lane& Road::getRightLane() {
    return rightLane;
}

Lane& Road::getLeftLane() {
    return leftLane;
}