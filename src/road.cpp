#include "road.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {
constexpr float LANE_WAYPOINT_SPACING = 40.0f;
constexpr float DASH_LENGTH = 60.0f;
constexpr float DASH_GAP = 60.0f;
constexpr float DASH_WIDTH = 6.0f;
constexpr float VEHICLE_LANE_CENTER_OFFSET = 40.0f;
constexpr float BIKE_LANE_WIDTH = 30.0f;
constexpr float MARKING_SAMPLE_SPACING = 20.0f;
constexpr Color BIKE_LANE_COLOR = { 46, 79, 76, 255 };
constexpr Color BIKE_LANE_MARKING = { 125, 190, 178, 255 };
constexpr Color SIDEWALK_COLOR = { 174, 170, 157, 255 };
constexpr Color SIDEWALK_ALT_COLOR = { 164, 161, 150, 255 };
constexpr Color SIDEWALK_JOINT_COLOR = { 127, 127, 119, 220 };
constexpr Color CURB_COLOR = { 211, 207, 193, 255 };
constexpr std::size_t SIDEWALK_SLAB_SEGMENTS = 4;
constexpr float SIDEWALK_JOINT_WIDTH = 3.0f;
constexpr float CURB_WIDTH = 5.0f;
constexpr float SIDEWALK_INTERSECTION_CLIP_DISTANCE = 22.0f;
constexpr float SIDEWALK_ROAD_CLEARANCE = 20.0f;

Vector2 add(Vector2 first, Vector2 second) {
    return { first.x + second.x, first.y + second.y };
}

Vector2 scale(Vector2 value, float amount) {
    return { value.x * amount, value.y * amount };
}

Vector2 interpolate(Vector2 start, Vector2 end, float amount) {
    return add(start, scale(Vector2Subtract(end, start), amount));
}

Rectangle pavedBoundsFor(const std::vector<Vector2>& centerline, float roadWidth) {
    if (centerline.empty()) {
        return { 0.0f, 0.0f, 0.0f, 0.0f };
    }

    float minimumX = centerline.front().x;
    float maximumX = centerline.front().x;
    float minimumY = centerline.front().y;
    float maximumY = centerline.front().y;
    for (const Vector2 point : centerline) {
        minimumX = std::min(minimumX, point.x);
        maximumX = std::max(maximumX, point.x);
        minimumY = std::min(minimumY, point.y);
        maximumY = std::max(maximumY, point.y);
    }

    const float halfWidth = roadWidth * 0.5f;
    return {
        minimumX - halfWidth,
        minimumY - halfWidth,
        maximumX - minimumX + roadWidth,
        maximumY - minimumY + roadWidth
    };
}

bool containsWithMargin(Rectangle bounds, Vector2 point, float margin) {
    return point.x >= bounds.x - margin &&
        point.x <= bounds.x + bounds.width + margin &&
        point.y >= bounds.y - margin &&
        point.y <= bounds.y + bounds.height + margin;
}

bool isInsideIntersection(Vector2 point, const std::vector<Intersection>& intersections) {
    return std::any_of(
        intersections.begin(),
        intersections.end(),
        [&](const Intersection& intersection) {
            return intersection.containsPoint(point);
        }
    );
}

bool isNearIntersection(Vector2 point, const std::vector<Intersection>& intersections, float distance) {
    return std::any_of(
        intersections.begin(),
        intersections.end(),
        [&](const Intersection& intersection) {
            return intersection.containsPointWithinDistance(point, distance);
        }
    );
}

bool overlapsAnotherRoad(
    Vector2 point,
    const Road& currentRoad,
    const std::vector<Road>& roads
) {
    for (const Road& otherRoad : roads) {
        if (&otherRoad == &currentRoad) {
            continue;
        }

        if (!containsWithMargin(otherRoad.getPavedBounds(), point, SIDEWALK_ROAD_CLEARANCE)) {
            continue;
        }

        const std::vector<Vector2>& centerline = otherRoad.getCenterline();
        const float corridorRadius = otherRoad.getWidth() * 0.5f + SIDEWALK_ROAD_CLEARANCE;
        const float corridorRadiusSquared = corridorRadius * corridorRadius;
        for (std::size_t index = 1; index < centerline.size(); ++index) {
            const Vector2 start = centerline[index - 1];
            const Vector2 segment = Vector2Subtract(centerline[index], start);
            const float segmentLengthSquared = Vector2LengthSqr(segment);
            const float projection = segmentLengthSquared <= 0.0001f
                ? 0.0f
                : std::clamp(
                    Vector2DotProduct(Vector2Subtract(point, start), segment) / segmentLengthSquared,
                    0.0f,
                    1.0f
                );
            const Vector2 closestPoint = Vector2Add(start, Vector2Scale(segment, projection));
            if (Vector2LengthSqr(Vector2Subtract(point, closestPoint)) <= corridorRadiusSquared) {
                return true;
            }
        }
    }

    return false;
}

void drawFrontFacingTriangle(Vector2 first, Vector2 second, Vector2 third, Color color) {
    const float signedArea =
        (second.x - first.x) * (third.y - first.y) -
        (third.x - first.x) * (second.y - first.y);
    // Raylib's 2D projection flips y and backface culling is enabled, so
    // world-space clockwise triangles are the visible winding.
    if (signedArea > 0.0f) {
        std::swap(second, third);
    }
    DrawTriangle(first, second, third, color);
}

void drawStrip(const std::vector<Vector2>& firstEdge, const std::vector<Vector2>& secondEdge, Color color) {
    for (std::size_t index = 1; index < firstEdge.size(); ++index) {
        drawFrontFacingTriangle(firstEdge[index - 1], firstEdge[index], secondEdge[index], color);
        drawFrontFacingTriangle(firstEdge[index - 1], secondEdge[index], secondEdge[index - 1], color);
    }
}

void drawSidewalk(
    const std::vector<Vector2>& innerEdge,
    const std::vector<Vector2>& outerEdge,
    const Road& currentRoad,
    const std::vector<Road>& roads,
    const std::vector<Intersection>& intersections
) {
    for (std::size_t index = 1; index < innerEdge.size(); ++index) {
        const Vector2 slabCenter = Vector2Scale(
            Vector2Add(
                Vector2Add(innerEdge[index - 1], innerEdge[index]),
                Vector2Add(outerEdge[index - 1], outerEdge[index])
            ),
            0.25f
        );
        const bool overlapsIntersection = isNearIntersection(
            slabCenter,
            intersections,
            SIDEWALK_INTERSECTION_CLIP_DISTANCE
        );
        if (!overlapsIntersection && !overlapsAnotherRoad(slabCenter, currentRoad, roads)) {
            const std::size_t slabIndex = (index - 1) / SIDEWALK_SLAB_SEGMENTS;
            const Color slabColor = slabIndex % 2 == 0 ? SIDEWALK_COLOR : SIDEWALK_ALT_COLOR;
            drawFrontFacingTriangle(innerEdge[index - 1], innerEdge[index], outerEdge[index], slabColor);
            drawFrontFacingTriangle(innerEdge[index - 1], outerEdge[index], outerEdge[index - 1], slabColor);
        }

        if (index % SIDEWALK_SLAB_SEGMENTS == 0) {
            const Vector2 jointCenter = Vector2Scale(Vector2Add(innerEdge[index], outerEdge[index]), 0.5f);
            if (!isNearIntersection(jointCenter, intersections, SIDEWALK_INTERSECTION_CLIP_DISTANCE) &&
                !overlapsAnotherRoad(jointCenter, currentRoad, roads)) {
                DrawLineEx(innerEdge[index], outerEdge[index], SIDEWALK_JOINT_WIDTH, SIDEWALK_JOINT_COLOR);
            }
        }
    }
}

void drawOffsetMarking(
    const std::vector<Vector2>& points,
    const std::vector<Intersection>& intersections,
    Color color,
    float thickness
) {
    for (std::size_t index = 1; index < points.size(); ++index) {
        const Vector2 start = points[index - 1];
        const Vector2 end = points[index];
        const float length = Vector2Distance(start, end);
        const int sampleCount = std::max(1, static_cast<int>(std::ceil(length / MARKING_SAMPLE_SPACING)));
        for (int sample = 0; sample < sampleCount; ++sample) {
            const float startAmount = static_cast<float>(sample) / static_cast<float>(sampleCount);
            const float endAmount = static_cast<float>(sample + 1) / static_cast<float>(sampleCount);
            const Vector2 pieceStart = interpolate(start, end, startAmount);
            const Vector2 pieceEnd = interpolate(start, end, endAmount);
            if (!isInsideIntersection(interpolate(pieceStart, pieceEnd, 0.5f), intersections)) {
                DrawLineEx(pieceStart, pieceEnd, thickness, color);
            }
        }
    }
}
}

Road::Road(float centerX, float centerY, float heading, float width, float length)
    : Road(makeStraightCenterline(centerX, centerY, heading, length), width) {}

Road::Road(std::vector<Vector2> points, float roadWidth)
    : centerline(std::move(points)),
      width(roadWidth),
      pavedBounds(pavedBoundsFor(centerline, width)),
      rightLane(makeLaneWaypoints(centerline, VEHICLE_LANE_CENTER_OFFSET, false)),
      leftLane(makeLaneWaypoints(centerline, -VEHICLE_LANE_CENTER_OFFSET, true)),
      rightBikeLane(makeLaneWaypoints(centerline, width * 0.5f - BIKE_LANE_WIDTH * 0.5f, false)),
      leftBikeLane(makeLaneWaypoints(centerline, -width * 0.5f + BIKE_LANE_WIDTH * 0.5f, true)),
      rightSidewalk(makeLaneWaypoints(centerline, width * 0.5f + SIDEWALK_WIDTH * 0.5f, false)),
      leftSidewalk(makeLaneWaypoints(centerline, -width * 0.5f - SIDEWALK_WIDTH * 0.5f, true)) {}

void Road::Draw(
    const std::vector<Intersection>& intersections,
    const std::vector<Road>& roads
) const {
    const std::vector<Vector2> positiveEdge = offsetPolyline(centerline, width * 0.5f);
    const std::vector<Vector2> negativeEdge = offsetPolyline(centerline, -width * 0.5f);
    drawStrip(positiveEdge, negativeEdge, DARKGRAY);

    const float bikeLaneInnerOffset = width * 0.5f - BIKE_LANE_WIDTH;
    const std::vector<Vector2> rightBikeInnerEdge = offsetPolyline(centerline, bikeLaneInnerOffset);
    const std::vector<Vector2> leftBikeInnerEdge = offsetPolyline(centerline, -bikeLaneInnerOffset);
    drawStrip(rightBikeInnerEdge, positiveEdge, BIKE_LANE_COLOR);
    drawStrip(leftBikeInnerEdge, negativeEdge, BIKE_LANE_COLOR);

    const std::vector<Vector2> rightSidewalkOuterEdge = offsetPolyline(
        centerline,
        width * 0.5f + SIDEWALK_WIDTH
    );
    const std::vector<Vector2> leftSidewalkOuterEdge = offsetPolyline(
        centerline,
        -width * 0.5f - SIDEWALK_WIDTH
    );
    drawSidewalk(positiveEdge, rightSidewalkOuterEdge, *this, roads, intersections);
    drawSidewalk(negativeEdge, leftSidewalkOuterEdge, *this, roads, intersections);
    drawOffsetMarking(positiveEdge, intersections, CURB_COLOR, CURB_WIDTH);
    drawOffsetMarking(negativeEdge, intersections, CURB_COLOR, CURB_WIDTH);

    drawOffsetMarking(
        rightBikeInnerEdge, intersections, WHITE, 4.0f
    );
    drawOffsetMarking(
        leftBikeInnerEdge, intersections, WHITE, 4.0f
    );
    drawOffsetMarking(
        positiveEdge, intersections, BIKE_LANE_MARKING, 4.0f
    );
    drawOffsetMarking(
        negativeEdge, intersections, BIKE_LANE_MARKING, 4.0f
    );

    const float dashPeriod = DASH_LENGTH + DASH_GAP;
    float distanceAlongRoad = 0.0f;

    for (std::size_t index = 1; index < centerline.size(); ++index) {
        const Vector2 segmentStart = centerline[index - 1];
        const Vector2 segmentEnd = centerline[index];
        const float segmentLength = Vector2Distance(segmentStart, segmentEnd);
        if (segmentLength <= 0.0001f) {
            continue;
        }

        float localDistance = 0.0f;
        while (localDistance < segmentLength) {
            const float phase = std::fmod(distanceAlongRoad + localDistance, dashPeriod);
            const bool insideDash = phase < DASH_LENGTH;
            const float distanceToBoundary = insideDash
                ? DASH_LENGTH - phase
                : dashPeriod - phase;
            const float pieceLength = std::min(segmentLength - localDistance, distanceToBoundary);

            if (insideDash && pieceLength > 0.0001f) {
                const Vector2 dashStart = interpolate(segmentStart, segmentEnd, localDistance / segmentLength);
                const Vector2 dashEnd = interpolate(
                    segmentStart,
                    segmentEnd,
                    (localDistance + pieceLength) / segmentLength
                );
                const Vector2 dashCenter = interpolate(dashStart, dashEnd, 0.5f);

                if (!isInsideIntersection(dashCenter, intersections)) {
                    DrawLineEx(dashStart, dashEnd, DASH_WIDTH, YELLOW);
                }
            }

            localDistance += std::max(pieceLength, 0.0001f);
        }

        distanceAlongRoad += segmentLength;
    }
}

Lane& Road::getRightLane() { return rightLane; }
const Lane& Road::getRightLane() const { return rightLane; }
Lane& Road::getLeftLane() { return leftLane; }
const Lane& Road::getLeftLane() const { return leftLane; }

float Road::getWidth() const {
    return width;
}

Rectangle Road::getPavedBounds() const {
    return pavedBounds;
}

const std::vector<Vector2>& Road::getCenterline() const {
    return centerline;
}

const std::vector<Vector2>& Road::getRightBikeLane() const { return rightBikeLane; }
const std::vector<Vector2>& Road::getLeftBikeLane() const { return leftBikeLane; }
const std::vector<Vector2>& Road::getRightSidewalk() const { return rightSidewalk; }
const std::vector<Vector2>& Road::getLeftSidewalk() const { return leftSidewalk; }

std::vector<Vector2> Road::makeStraightCenterline(
    float centerX,
    float centerY,
    float heading,
    float length
) {
    const Vector2 direction = { std::cos(heading), std::sin(heading) };
    return {
        { centerX - direction.x * length / 2.0f, centerY - direction.y * length / 2.0f },
        { centerX + direction.x * length / 2.0f, centerY + direction.y * length / 2.0f }
    };
}

std::vector<Vector2> Road::offsetPolyline(const std::vector<Vector2>& points, float offset) {
    std::vector<Vector2> offsetPoints;
    offsetPoints.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index) {
        const std::size_t previous = index == 0 ? index : index - 1;
        const std::size_t next = index + 1 < points.size() ? index + 1 : index;
        Vector2 direction = Vector2Subtract(points[next], points[previous]);
        const float directionLength = Vector2Length(direction);

        if (directionLength <= 0.0001f) {
            offsetPoints.push_back(points[index]);
            continue;
        }

        direction = Vector2Scale(direction, 1.0f / directionLength);
        const Vector2 normal = { -direction.y, direction.x };
        offsetPoints.push_back(Vector2Add(points[index], Vector2Scale(normal, offset)));
    }

    return offsetPoints;
}

std::vector<Vector2> Road::resamplePolyline(const std::vector<Vector2>& points, float spacing) {
    if (points.size() < 2 || spacing <= 0.0f) {
        return points;
    }

    std::vector<Vector2> sampled;
    sampled.reserve(points.size());
    sampled.push_back(points.front());

    float totalDistance = 0.0f;
    float nextSampleDistance = spacing;

    for (std::size_t index = 1; index < points.size(); ++index) {
        const Vector2 start = points[index - 1];
        const Vector2 end = points[index];
        const float segmentLength = Vector2Distance(start, end);
        if (segmentLength <= 0.0001f) {
            continue;
        }

        while (nextSampleDistance <= totalDistance + segmentLength) {
            const float amount = (nextSampleDistance - totalDistance) / segmentLength;
            sampled.push_back(interpolate(start, end, amount));
            nextSampleDistance += spacing;
        }

        totalDistance += segmentLength;
    }

    if (Vector2Distance(sampled.back(), points.back()) > 0.001f) {
        sampled.push_back(points.back());
    }

    return sampled;
}

std::vector<Vector2> Road::makeLaneWaypoints(
    const std::vector<Vector2>& points,
    float offset,
    bool reverse
) {
    std::vector<Vector2> lanePoints = resamplePolyline(offsetPolyline(points, offset), LANE_WAYPOINT_SPACING);
    if (reverse) {
        std::reverse(lanePoints.begin(), lanePoints.end());
    }
    return lanePoints;
}
