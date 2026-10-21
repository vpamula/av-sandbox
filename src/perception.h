#pragma once

#include "vehicle.h"

#include <random>
#include <vector>

class World;

enum class PerceivedAgentType {
    Vehicle,
    Bicycle,
    Pedestrian
};

struct PerceptionTarget {
    int id = 0;
    PerceivedAgentType type = PerceivedAgentType::Vehicle;
    Vector2 position = { 0.0f, 0.0f };
    float heading = 0.0f;
    float speed = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
};

struct PerceivedObject {
    int id = 0;
    PerceivedAgentType type = PerceivedAgentType::Vehicle;
    Vector2 measuredPosition = { 0.0f, 0.0f };
    float distance = 0.0f;
    float bearing = 0.0f;
    float heading = 0.0f;
    float speed = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
};

class Perception {
public:
    Perception();

    void update(const Vehicle& ego, const std::vector<PerceptionTarget>& targets, const World& world);
    const std::vector<PerceivedObject>& getObjects() const;
    float getRecommendedEgoSpeed(const Vehicle& ego) const;
    void Draw(const Vehicle& ego) const;

private:
    static Color colorForType(PerceivedAgentType type);

    // The downtown blocks are 500 world units apart. Keep the sensor to 1.4 blocks.
    float detectionRange = 700.0f;
    float fieldOfViewRadians = 110.0f * DEG2RAD;
    std::mt19937 random;
    std::normal_distribution<float> positionNoise;
    std::vector<PerceivedObject> objects;
};
