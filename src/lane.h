#pragma once

#include "raylib.h"
#include <vector>

class Lane {
public:
    Lane(float centerX, float centerY, float heading, float length);

    std::vector<Vector2> generateWaypoints() const;
    Vector2 getDirection() const;
    Vector2 getCenter() const { return { centerX, centerY }; }

    Vector2 getStartPoint() const;
    Vector2 getEndPoint() const;

private:
    float centerX;
    float centerY;
    float heading;
    float length;
};
