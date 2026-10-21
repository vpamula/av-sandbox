#pragma once

#include "intersection.h"
#include "lane.h"
#include "raylib.h"

#include <vector>

class Road {
public:
    Road(float centerX, float centerY, float heading, float width, float length);

    void Draw(const std::vector<Intersection>& intersections) const;
    Lane& getRightLane();
    const Lane& getRightLane() const;
    Lane& getLeftLane();
    const Lane& getLeftLane() const;
    Rectangle getBounds() const;

private:
    float centerX;
    float centerY;
    float heading;
    float width;
    float length;
    Lane rightLane;
    Lane leftLane;
};
