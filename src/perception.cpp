#include "perception.h"

#include "raymath.h"
#include "world.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float POSITION_NOISE_STDDEV = 2.5f;
constexpr float FOLLOWING_DECELERATION = 500.0f;
constexpr float FOLLOWING_DISTANCE = 24.0f;
constexpr float FORWARD_BEARING_LIMIT = 18.0f * DEG2RAD;
constexpr float LATERAL_CLEARANCE = 12.0f;
}

Perception::Perception()
    : random(std::random_device{}()),
      positionNoise(0.0f, POSITION_NOISE_STDDEV) {}

void Perception::update(
    const Vehicle& ego,
    const std::vector<PerceptionTarget>& targets,
    const World& world
) {
    objects.clear();
    objects.reserve(targets.size());

    const Vector2 egoPosition = { ego.getX(), ego.getY() };
    const Vector2 forward = { std::cos(ego.getHeading()), std::sin(ego.getHeading()) };
    const float halfFieldOfView = fieldOfViewRadians * 0.5f;

    for (const PerceptionTarget& target : targets) {
        const Vector2 offset = Vector2Subtract(target.position, egoPosition);
        const float distance = Vector2Length(offset);
        if (distance > detectionRange || distance <= 0.001f) {
            continue;
        }

        const float bearing = std::atan2(
            forward.x * offset.y - forward.y * offset.x,
            Vector2DotProduct(forward, offset)
        );
        if (std::abs(bearing) > halfFieldOfView) {
            continue;
        }
        if (!world.hasRoadCorridorLineOfSight(egoPosition, target.position)) {
            continue;
        }

        objects.push_back({
            target.id,
            target.type,
            {
                target.position.x + positionNoise(random),
                target.position.y + positionNoise(random)
            },
            distance,
            bearing,
            target.heading,
            target.speed,
            target.length,
            target.width
        });
    }

    std::sort(objects.begin(), objects.end(), [](const PerceivedObject& first, const PerceivedObject& second) {
        return first.distance < second.distance;
    });
}

const std::vector<PerceivedObject>& Perception::getObjects() const {
    return objects;
}

float Perception::getRecommendedEgoSpeed(const Vehicle& ego) const {
    const float egoHeading = ego.getHeading();
    const Vector2 forward = { std::cos(egoHeading), std::sin(egoHeading) };
    float speedLimit = std::numeric_limits<float>::infinity();

    for (const PerceivedObject& object : objects) {
        const float lateralDistance = std::abs(object.distance * std::sin(object.bearing));
        const float lateralLimit = (ego.getWidth() + object.width) * 0.5f + LATERAL_CLEARANCE;
        if (std::abs(object.bearing) > FORWARD_BEARING_LIMIT || lateralDistance > lateralLimit) {
            continue;
        }

        const float longitudinalDistance = object.distance * std::cos(object.bearing);
        const float gap = longitudinalDistance -
            (ego.getLength() + object.length) * 0.5f - FOLLOWING_DISTANCE;
        if (gap <= 0.0f) {
            return 0.0f;
        }

        const Vector2 targetForward = { std::cos(object.heading), std::sin(object.heading) };
        const float targetLongitudinalSpeed = object.speed * Vector2DotProduct(targetForward, forward);
        const float safeClosingSpeed = std::sqrt(2.0f * FOLLOWING_DECELERATION * gap);
        speedLimit = std::min(speedLimit, std::max(0.0f, targetLongitudinalSpeed + safeClosingSpeed));
    }

    return speedLimit;
}

Color Perception::colorForType(PerceivedAgentType type) {
    switch (type) {
        case PerceivedAgentType::Bicycle: return SKYBLUE;
        case PerceivedAgentType::Pedestrian: return ORANGE;
        case PerceivedAgentType::Vehicle: return LIME;
    }
    return WHITE;
}

void Perception::Draw(const Vehicle& ego) const {
    const Vector2 egoPosition = { ego.getX(), ego.getY() };
    for (const PerceivedObject& object : objects) {
        const Color color = colorForType(object.type);
        DrawLineEx(egoPosition, object.measuredPosition, 1.5f, Fade(color, 0.35f));
        DrawCircleV(object.measuredPosition, 8.0f, Fade(color, 0.4f));
        DrawCircleLines(
            static_cast<int>(object.measuredPosition.x),
            static_cast<int>(object.measuredPosition.y),
            13.0f,
            color
        );
    }
}
