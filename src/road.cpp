#include "road.h"
#include <cmath>

Road::Road(float center_x, float center_y, float heading, float width, float length) 
: rightLane(center_x + cos(heading + PI/2) * width / 4, center_y + sin(heading + PI/2) * width / 4, heading, width / 2, length),
leftLane(center_x - cos(heading + PI/2) * width / 4, center_y - sin(heading + PI/2) * width / 4, heading + PI, width / 2, length) {
    this->center_x = center_x;
    this->center_y = center_y;
    this->heading = heading;
    this->width = width;
    this->length = length;
    rightLane.setParentRoad(this);
    leftLane.setParentRoad(this);
}

void Road::Draw(const std::vector<Intersection>& intersections) {
    Rectangle rect = {
        center_x,
        center_y,
        length,
        width
    };

    Vector2 origin = {
        length / 2,
        width / 2
    };

    DrawRectanglePro(
        rect,
        origin,
        heading * RAD2DEG,
        DARKGRAY
    );

    float dashLength = 80.0f;
    float dashGap = 60.0f;
    float dashWidth = 8.0f;

    Vector2 direction = {
        cos(heading),
        sin(heading)
    };

    Vector2 roadStart = {
        center_x -
        direction.x * length/2,

        center_y -
        direction.y * length/2
    };

    for (float s = dashLength / 2; s < length - dashLength / 2; s += dashLength + dashGap) {
        Vector2 center = {roadStart.x + direction.x * s, roadStart.y + direction.y * s};

        Rectangle dash = {
            center.x,
            center.y,
            dashLength,
            dashWidth
        };

        Vector2 dashOrigin = {
            dashLength / 2,
            dashWidth / 2
        };
    
        bool insideIntersection = false;

        for (const Intersection& intersection : intersections) {
            if (CheckCollisionPointRec(center, intersection.getBounds())) {
                insideIntersection = true;
                break;
            }
        }

        if (!insideIntersection) {
            DrawRectanglePro(
                dash,
                dashOrigin,
                heading * RAD2DEG,
                YELLOW
            );
        }
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

Rectangle Road::getBounds() {
    bool horizontal =
        fabs(sin(heading)) < 0.01f;
    
    if (horizontal) {
        return {
            center_x - length / 2,
            center_y - width / 2,
            length,
            width
        };
    }
    else {
        return {
            center_x - width / 2,
            center_y - length / 2,
            width,
            length
        };
    }
}