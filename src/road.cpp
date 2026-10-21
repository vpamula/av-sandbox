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

Vector2 add(Vector2 first, Vector2 second) {
    return { first.x + second.x, first.y + second.y };
}

Vector2 scale(Vector2 value, float amount) {
    return { value.x * amount, value.y * amount };
}

Vector2 interpolate(Vector2 start, Vector2 end, float amount) {
    return add(start, scale(Vector2Subtract(end, start), amount));
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
}

Road::Road(float centerX, float centerY, float heading, float width, float length)
    : Road(makeStraightCenterline(centerX, centerY, heading, length), width) {}

Road::Road(std::vector<Vector2> points, float roadWidth)
    : centerline(std::move(points)),
      width(roadWidth),
      rightLane(makeLaneWaypoints(centerline, width / 4.0f, false)),
      leftLane(makeLaneWaypoints(centerline, -width / 4.0f, true)) {}

void Road::Draw(const std::vector<Intersection>& intersections) const {
    const std::vector<Vector2> positiveEdge = offsetPolyline(centerline, width * 0.5f);
    const std::vector<Vector2> negativeEdge = offsetPolyline(centerline, -width * 0.5f);

    for (std::size_t index = 1; index < centerline.size(); ++index) {
        DrawTriangle(
            positiveEdge[index - 1],
            positiveEdge[index],
            negativeEdge[index],
            DARKGRAY
        );
        DrawTriangle(
            positiveEdge[index - 1],
            negativeEdge[index],
            negativeEdge[index - 1],
            DARKGRAY
        );
    }

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

const std::vector<Vector2>& Road::getCenterline() const {
    return centerline;
}

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
