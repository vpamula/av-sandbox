#pragma once

#include "intersection.h"
#include "lane.h"
#include "raylib.h"

#include <cstddef>
#include <vector>

class Road {
public:
    static constexpr float SIDEWALK_WIDTH = 30.0f;

    Road(
        float centerX,
        float centerY,
        float heading,
        float width,
        float length,
        std::size_t lanesPerDirection = 1
    );
    Road(std::vector<Vector2> centerline, float width, std::size_t lanesPerDirection = 1);

    void Draw(
        const std::vector<Intersection>& intersections,
        const std::vector<Road>& roads
    ) const;
    Lane& getRightLane();
    const Lane& getRightLane() const;
    Lane& getLeftLane();
    const Lane& getLeftLane() const;
    std::vector<Lane>& getRightLanes();
    const std::vector<Lane>& getRightLanes() const;
    std::vector<Lane>& getLeftLanes();
    const std::vector<Lane>& getLeftLanes() const;
    bool isRightmostLane(const Lane* lane) const;
    float getWidth() const;
    Rectangle getPavedBounds() const;
    const std::vector<Vector2>& getCenterline() const;
    const std::vector<Vector2>& getRightBikeLane() const;
    const std::vector<Vector2>& getLeftBikeLane() const;
    const std::vector<Vector2>& getRightSidewalk() const;
    const std::vector<Vector2>& getLeftSidewalk() const;

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
    static std::vector<Lane> makeDirectionalLanes(
        const std::vector<Vector2>& points,
        float roadWidth,
        std::size_t lanesPerDirection,
        bool reverse
    );

    std::vector<Vector2> centerline;
    float width;
    std::size_t lanesPerDirection;
    Rectangle pavedBounds;
    std::vector<Lane> rightLanes;
    std::vector<Lane> leftLanes;
    std::vector<Vector2> rightBikeLane;
    std::vector<Vector2> leftBikeLane;
    std::vector<Vector2> rightSidewalk;
    std::vector<Vector2> leftSidewalk;
};
