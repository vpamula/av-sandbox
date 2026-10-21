#include "npctraffic.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr float NPC_VEHICLE_LENGTH = 80.0f;
constexpr float NPC_VEHICLE_WIDTH = 40.0f;
constexpr float MIN_SPAWN_SEPARATION = 300.0f;
constexpr float MIN_ROUTE_LENGTH = 1000.0f;
constexpr float ARRIVAL_RADIUS = 55.0f;
constexpr float HEADWAY_DISTANCE = 24.0f;
constexpr float FOLLOWING_DECELERATION = 360.0f;
constexpr float FOLLOWING_ACCELERATION = 160.0f;

float routeLength(const std::vector<Vector2>& route) {
    float length = 0.0f;
    for (std::size_t index = 1; index < route.size(); ++index) {
        length += Vector2Distance(route[index - 1], route[index]);
    }
    return length;
}

Vector2 positionOf(const Vehicle& vehicle) {
    return { vehicle.getX(), vehicle.getY() };
}
}

NpcTraffic::Car::Car(Vector2 start, float heading, Color color)
    : vehicle(start.x, start.y, NPC_VEHICLE_LENGTH, NPC_VEHICLE_WIDTH, heading),
      color(color) {}

NpcTraffic::NpcTraffic(World& world, const Vehicle& player, std::size_t carCount)
    : random(std::random_device{}()) {
    const Color colors[] = {
        { 60, 190, 255, 255 },
        { 255, 165, 50, 255 },
        { 175, 105, 255, 255 },
        { 80, 225, 170, 255 },
        { 245, 220, 75, 255 },
        { 255, 110, 145, 255 }
    };

    for (std::size_t index = 0; index < carCount; ++index) {
        if (!spawnCar(world, player, colors[index % (sizeof(colors) / sizeof(colors[0]))])) {
            break;
        }
    }
}

Vector2 NpcTraffic::randomRoadPoint(const World& world) {
    const std::vector<Road>& roads = world.getRoads();
    if (roads.empty()) {
        return { 0.0f, 0.0f };
    }

    std::uniform_int_distribution<std::size_t> chooseRoad(0, roads.size() - 1);
    std::uniform_int_distribution<int> chooseLane(0, 1);
    Vector2 fallback = { 0.0f, 0.0f };

    for (int attempt = 0; attempt < 80; ++attempt) {
        const Road& road = roads[chooseRoad(random)];
        const Lane& lane = chooseLane(random) == 0 ? road.getRightLane() : road.getLeftLane();
        const std::vector<Vector2>& waypoints = lane.getWaypoints();
        if (waypoints.empty()) {
            continue;
        }

        std::uniform_int_distribution<std::size_t> chooseWaypoint(0, waypoints.size() - 1);
        fallback = waypoints[chooseWaypoint(random)];
        if (!world.isPointInIntersection(fallback)) {
            return fallback;
        }
    }

    return fallback;
}

std::vector<Vector2> NpcTraffic::buildRoute(World& world, Vector2 start, bool keepExactStart) {
    for (int attempt = 0; attempt < 60; ++attempt) {
        const Vector2 destination = randomRoadPoint(world);
        if (Vector2Distance(start, destination) < MIN_ROUTE_LENGTH) {
            continue;
        }

        std::vector<Vector2> route = world.buildRouteBetweenPoints(start, destination);
        if (route.size() < 2 || routeLength(route) < MIN_ROUTE_LENGTH) {
            continue;
        }

        if (keepExactStart) {
            route.front() = start;
        }
        return route;
    }

    return {};
}

bool NpcTraffic::spawnCar(World& world, const Vehicle& player, Color color) {
    for (int attempt = 0; attempt < 100; ++attempt) {
        const Vector2 start = randomRoadPoint(world);
        if (Vector2Distance(start, positionOf(player)) < MIN_SPAWN_SEPARATION) {
            continue;
        }

        const bool overlapsExistingCar = std::any_of(
            cars.begin(),
            cars.end(),
            [&](const Car& car) {
                return Vector2Distance(start, positionOf(car.vehicle)) < MIN_SPAWN_SEPARATION;
            }
        );
        if (overlapsExistingCar) {
            continue;
        }

        std::vector<Vector2> route = buildRoute(world, start, false);
        if (route.size() < 2) {
            continue;
        }

        const Vector2 firstDirection = Vector2Subtract(route[1], route[0]);
        const float heading = std::atan2(firstDirection.y, firstDirection.x);
        Car car(route.front(), heading, color);
        car.route = std::move(route);
        car.vehicle.setSpeed(150.0f);
        car.controller.setRouteControls(world.getRouteControlsAlongRoute(car.route, car.vehicle.getLength()));
        car.controller.setTrafficLightStates(world.getTrafficLightStates());
        car.controller.reacquireWaypoint(car.vehicle, car.route);
        cars.push_back(std::move(car));
        return true;
    }

    return false;
}

float NpcTraffic::speedLimitForTraffic(
    const Car& car,
    std::size_t carIndex,
    const Vehicle& player
) const {
    const Vector2 forward = { std::cos(car.vehicle.getHeading()), std::sin(car.vehicle.getHeading()) };
    const Vector2 position = positionOf(car.vehicle);
    float speedLimit = std::numeric_limits<float>::infinity();

    auto considerVehicleAhead = [&](const Vehicle& other) {
        const Vector2 offset = Vector2Subtract(positionOf(other), position);
        const float distanceAhead = Vector2DotProduct(offset, forward);
        const float lateralDistance = std::abs(forward.x * offset.y - forward.y * offset.x);
        const Vector2 otherForward = { std::cos(other.getHeading()), std::sin(other.getHeading()) };
        if (distanceAhead <= 0.0f || lateralDistance > 52.0f ||
            Vector2DotProduct(forward, otherForward) < 0.65f) {
            return;
        }

        const float gap = distanceAhead - (car.vehicle.getLength() + other.getLength()) * 0.5f - HEADWAY_DISTANCE;
        const float safeGap = std::max(0.0f, gap);
        const float safeSpeed = std::sqrt(2.0f * FOLLOWING_DECELERATION * safeGap);
        speedLimit = std::min(speedLimit, safeSpeed);
    };

    considerVehicleAhead(player);
    for (std::size_t index = 0; index < cars.size(); ++index) {
        if (index != carIndex) {
            considerVehicleAhead(cars[index].vehicle);
        }
    }

    return speedLimit;
}

void NpcTraffic::prepareForStep(World& world, const Vehicle& player, float dt) {
    const std::vector<TrafficLightState> trafficLightStates = world.getTrafficLightStates();

    for (std::size_t index = 0; index < cars.size(); ++index) {
        Car& car = cars[index];
        if (car.route.size() < 2) {
            continue;
        }

        if (Vector2Distance(positionOf(car.vehicle), car.route.back()) <= ARRIVAL_RADIUS) {
            std::vector<Vector2> nextRoute = buildRoute(world, positionOf(car.vehicle), true);
            if (nextRoute.size() < 2) {
                car.vehicle.setSpeed(0.0f);
                car.vehicle.setSteeringAngle(0.0f);
                continue;
            }

            car.route = std::move(nextRoute);
            car.controller.setRouteControls(world.getRouteControlsAlongRoute(car.route, car.vehicle.getLength()));
            car.controller.reacquireWaypoint(car.vehicle, car.route);
        }

        car.controller.setTrafficLightStates(trafficLightStates);
        car.controller.update(car.vehicle, car.route, dt);

        const float trafficSpeedLimit = speedLimitForTraffic(car, index, player);
        if (trafficSpeedLimit < car.vehicle.getSpeed()) {
            car.vehicle.approachSpeed(
                trafficSpeedLimit,
                FOLLOWING_ACCELERATION,
                FOLLOWING_DECELERATION,
                dt
            );
        }

    }
}

std::vector<Vehicle*> NpcTraffic::getVehicles() {
    std::vector<Vehicle*> vehicles;
    vehicles.reserve(cars.size());
    for (Car& car : cars) {
        vehicles.push_back(&car.vehicle);
    }
    return vehicles;
}

std::vector<StopSignParticipant> NpcTraffic::getStopSignParticipants() {
    std::vector<StopSignParticipant> participants;
    participants.reserve(cars.size());
    for (Car& car : cars) {
        participants.push_back({ &car.vehicle, &car.controller });
    }
    return participants;
}

void NpcTraffic::Draw() const {
    for (const Car& car : cars) {
        car.vehicle.Draw(car.color);
    }
}

std::size_t NpcTraffic::size() const {
    return cars.size();
}
