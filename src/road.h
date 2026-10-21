#pragma once

#include "intersection.h"
#include "lane.h"
#include "raylib.h"

#include <vector>

class Road {
public:
    Road(float centerX, float centerY, float heading, float width, float length);
    Road(std::vector<Vector2> centerline, float width);

    void Draw(const std::vector<Intersection>& intersections) const;
    Lane& getRightLane();
    const Lane& getRightLane() const;
    Lane& getLeftLane();
    const Lane& getLeftLane() const;
    float getWidth() const;
    const std::vector<Vector2>& getCenterline() const;

private:
    static std::vector<Vector2> makeStraightCenterline(
        float centerX,
        float centerY,
        float heading,
        float length
    );
    static std::vector<Vector2> offsetPolyline(const std::vector<Vector2>& points, float offset);
    static std::vector<Vector2> resamplePolyline(const std::vector<Vector2>& points, float spacing);
    static std::vector<Vector2> makeLaneWaypoints(
        const std::vector<Vector2>& points,
        float offset,
        bool reverse
    );

    std::vector<Vector2> centerline;
    float width;
    Lane rightLane;
    Lane leftLane;
};
