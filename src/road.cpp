#include "road.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float DASH_LENGTH = 80.0f;
constexpr float DASH_GAP = 60.0f;
constexpr float DASH_WIDTH = 8.0f;
}

Road::Road(float centerX, float centerY, float heading, float width, float length)
    : centerX(centerX),
      centerY(centerY),
      heading(heading),
      width(width),
      length(length),
      rightLane(
          centerX + std::cos(heading + PI / 2.0f) * width / 4.0f,
          centerY + std::sin(heading + PI / 2.0f) * width / 4.0f,
          heading,
          length
      ),
      leftLane(
          centerX - std::cos(heading + PI / 2.0f) * width / 4.0f,
          centerY - std::sin(heading + PI / 2.0f) * width / 4.0f,
          heading + PI,
          length
      ) {}

void Road::Draw(const std::vector<Intersection>& intersections) const {
    DrawRectanglePro(
        { centerX, centerY, length, width },
        { length / 2.0f, width / 2.0f },
        heading * RAD2DEG,
        DARKGRAY
    );

    const Vector2 direction = { std::cos(heading), std::sin(heading) };
    const Vector2 roadStart = {
        centerX - direction.x * length / 2.0f,
        centerY - direction.y * length / 2.0f
    };

    for (float distance = DASH_LENGTH / 2.0f;
         distance < length - DASH_LENGTH / 2.0f;
         distance += DASH_LENGTH + DASH_GAP) {
        const Vector2 center = {
            roadStart.x + direction.x * distance,
            roadStart.y + direction.y * distance
        };

        const bool insideIntersection = std::any_of(
            intersections.begin(),
            intersections.end(),
            [&](const Intersection& intersection) {
                return CheckCollisionPointRec(center, intersection.getBounds());
            }
        );

        if (!insideIntersection) {
            DrawRectanglePro(
                { center.x, center.y, DASH_LENGTH, DASH_WIDTH },
                { DASH_LENGTH / 2.0f, DASH_WIDTH / 2.0f },
                heading * RAD2DEG,
                YELLOW
            );
        }
    }
}

Lane& Road::getRightLane() { return rightLane; }
const Lane& Road::getRightLane() const { return rightLane; }
Lane& Road::getLeftLane() { return leftLane; }
const Lane& Road::getLeftLane() const { return leftLane; }

Rectangle Road::getBounds() const {
    const float halfWidth = width / 2.0f;
    const float halfLength = length / 2.0f;
    const float extentX = std::abs(std::cos(heading)) * halfLength + std::abs(std::sin(heading)) * halfWidth;
    const float extentY = std::abs(std::sin(heading)) * halfLength + std::abs(std::cos(heading)) * halfWidth;

    return {
        centerX - extentX,
        centerY - extentY,
        extentX * 2.0f,
        extentY * 2.0f
    };
}
