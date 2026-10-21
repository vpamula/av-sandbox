#pragma once

#include "lanefollower.h"
#include "stopsigncoordinator.h"
#include "vehicle.h"
#include "world.h"

#include <cstddef>
#include <random>
#include <vector>

class NpcTraffic {
public:
    NpcTraffic(World& world, const Vehicle& player, std::size_t carCount);

    void prepareForStep(World& world, const Vehicle& player, float dt);
    std::vector<Vehicle*> getVehicles();
    std::vector<StopSignParticipant> getStopSignParticipants();
    void Draw() const;
    std::size_t size() const;

private:
    struct Car {
        Car(Vector2 start, float heading, Color color);

        Vehicle vehicle;
        LaneFollower controller;
        std::vector<Vector2> route;
        Color color;
    };

    Vector2 randomRoadPoint(const World& world);
    std::vector<Vector2> buildRoute(World& world, Vector2 start, bool keepExactStart);
    bool spawnCar(World& world, const Vehicle& player, Color color);
    float speedLimitForTraffic(const Car& car, std::size_t carIndex, const Vehicle& player) const;

    std::vector<Car> cars;
    std::mt19937 random;
};
