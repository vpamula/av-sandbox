#pragma once

#include "lanefollower.h"
#include "vehicle.h"
#include "world.h"

#include <cstddef>
#include <random>
#include <vector>

enum class RoadUserType {
    Bicycle,
    Pedestrian
};

class NpcRoadUsers {
public:
    NpcRoadUsers(
        World& world,
        const Vehicle& ego,
        const std::vector<Vehicle*>& otherVehicles,
        std::size_t bicycleCount,
        std::size_t pedestrianCount
    );

    void prepareForStep(float dt);
    std::vector<Vehicle*> getVehicles();
    void Draw() const;
    std::size_t bicycleCount() const;
    std::size_t pedestrianCount() const;

private:
    struct Agent {
        Agent(
            Vector2 start,
            float heading,
            RoadUserType type,
            Color color,
            bool usesCrosswalk
        );

        Vehicle vehicle;
        LaneFollower controller;
        std::vector<Vector2> route;
        RoadUserType type;
        Color color;
        bool usesCrosswalk;
        float turnaroundPause = 0.0f;
    };

    bool spawnAgent(
        World& world,
        RoadUserType type,
        Color color,
        bool useCrosswalk,
        std::vector<Vector2>& occupiedPositions
    );

    std::vector<Agent> agents;
    std::mt19937 random;
};
