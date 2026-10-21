#include "npcroadusers.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {
constexpr float BICYCLE_LENGTH = 34.0f;
constexpr float BICYCLE_WIDTH = 15.0f;
constexpr float PEDESTRIAN_LENGTH = 14.0f;
constexpr float PEDESTRIAN_WIDTH = 14.0f;
constexpr float MINIMUM_SPAWN_SEPARATION = 150.0f;
constexpr float TURNAROUND_DISTANCE = 38.0f;
constexpr float BICYCLE_TURNAROUND_PAUSE = 0.8f;
constexpr float PEDESTRIAN_TURNAROUND_PAUSE = 0.45f;
constexpr float CROSSWALK_TURNAROUND_PAUSE = 2.0f;

float distanceAlong(const std::vector<Vector2>& points, std::size_t startIndex) {
    float length = 0.0f;
    for (std::size_t index = startIndex + 1; index < points.size(); ++index) {
        length += Vector2Distance(points[index - 1], points[index]);
    }
    return length;
}

float routeLength(const std::vector<Vector2>& points) {
    return distanceAlong(points, 0);
}

Vector2 positionOf(const Vehicle& vehicle) {
    return { vehicle.getX(), vehicle.getY() };
}

float headingOf(Vector2 from, Vector2 to) {
    const Vector2 direction = Vector2Subtract(to, from);
    return std::atan2(direction.y, direction.x);
}

void truncateSidewalkRouteBeforeIntersection(const World& world, std::vector<Vector2>& route) {
    constexpr float JUNCTION_SAMPLE_SPACING = 10.0f;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const Vector2 start = route[index - 1];
        const Vector2 end = route[index];
        const int sampleCount = std::max(
            1,
            static_cast<int>(std::ceil(Vector2Distance(start, end) / JUNCTION_SAMPLE_SPACING))
        );
        for (int sample = 1; sample <= sampleCount; ++sample) {
            const float amount = static_cast<float>(sample) / static_cast<float>(sampleCount);
            const Vector2 point = Vector2Add(start, Vector2Scale(Vector2Subtract(end, start), amount));
            if (world.isPointInIntersection(point)) {
                route.resize(index);
                return;
            }
        }
    }
}
}

NpcRoadUsers::Agent::Agent(
    Vector2 start,
    float heading,
    RoadUserType type,
    Color color,
    bool usesCrosswalk
)
    : vehicle(
          start.x,
          start.y,
          type == RoadUserType::Bicycle ? BICYCLE_LENGTH : PEDESTRIAN_LENGTH,
          type == RoadUserType::Bicycle ? BICYCLE_WIDTH : PEDESTRIAN_WIDTH,
          heading
      ),
      controller(
          type == RoadUserType::Bicycle ? 85.0f : 22.0f,
          type == RoadUserType::Bicycle ? 48.0f : 15.0f,
          type == RoadUserType::Bicycle ? 35.0f : 18.0f
      ),
      type(type),
      color(color),
      usesCrosswalk(usesCrosswalk) {}

NpcRoadUsers::NpcRoadUsers(
    World& world,
    const Vehicle& ego,
    const std::vector<Vehicle*>& otherVehicles,
    std::size_t bicycleCount,
    std::size_t pedestrianCount
)
    : random(std::random_device{}()) {
    std::vector<Vector2> occupiedPositions = { positionOf(ego) };
    occupiedPositions.reserve(otherVehicles.size() + bicycleCount + pedestrianCount + 1);
    for (const Vehicle* other : otherVehicles) {
        if (other) {
            occupiedPositions.push_back(positionOf(*other));
        }
    }

    const Color bicycleColors[] = {
        { 55, 205, 235, 255 },
        { 245, 190, 55, 255 },
        { 220, 90, 170, 255 },
        { 100, 220, 120, 255 }
    };
    const Color pedestrianColors[] = {
        { 255, 135, 70, 255 },
        { 245, 225, 105, 255 },
        { 190, 135, 255, 255 },
        { 250, 105, 110, 255 }
    };

    for (std::size_t index = 0; index < bicycleCount; ++index) {
        if (!spawnAgent(
                world,
                RoadUserType::Bicycle,
                bicycleColors[index % (sizeof(bicycleColors) / sizeof(bicycleColors[0]))],
                false,
                occupiedPositions
            )) {
            break;
        }
    }
    const std::size_t crossingPedestrianCount = pedestrianCount == 0
        ? 0
        : std::max<std::size_t>(1, pedestrianCount / 3);
    for (std::size_t index = 0; index < pedestrianCount; ++index) {
        if (!spawnAgent(
                world,
                RoadUserType::Pedestrian,
                pedestrianColors[index % (sizeof(pedestrianColors) / sizeof(pedestrianColors[0]))],
                index < crossingPedestrianCount,
                occupiedPositions
            )) {
            break;
        }
    }
}

bool NpcRoadUsers::spawnAgent(
    World& world,
    RoadUserType type,
    Color color,
    bool useCrosswalk,
    std::vector<Vector2>& occupiedPositions
) {
    const std::vector<Road>& roads = world.getRoads();
    if (roads.empty()) {
        return false;
    }

    std::uniform_int_distribution<std::size_t> chooseRoad(0, roads.size() - 1);
    std::uniform_int_distribution<int> chooseSide(0, 1);
    const std::vector<std::vector<Vector2>> pedestrianCrossings = useCrosswalk
        ? world.getPedestrianCrossings()
        : std::vector<std::vector<Vector2>>{};
    if (useCrosswalk && pedestrianCrossings.empty()) {
        return false;
    }
    std::uniform_int_distribution<std::size_t> chooseCrossing(
        0,
        pedestrianCrossings.empty() ? 0 : pedestrianCrossings.size() - 1
    );
    std::uniform_int_distribution<int> chooseDirection(0, 1);

    for (int attempt = 0; attempt < 120; ++attempt) {
        std::vector<Vector2> route;
        if (useCrosswalk) {
            route = pedestrianCrossings[chooseCrossing(random)];
            if (chooseDirection(random) == 1) {
                std::reverse(route.begin(), route.end());
            }
        }
        else {
            const Road& road = roads[chooseRoad(random)];
            const int side = chooseSide(random);
            const std::vector<Vector2>* path = nullptr;
            if (type == RoadUserType::Bicycle) {
                path = side == 0 ? &road.getRightBikeLane() : &road.getLeftBikeLane();
            }
            else {
                path = side == 0 ? &road.getRightSidewalk() : &road.getLeftSidewalk();
            }

            if (!path || path->size() < 3) {
                continue;
            }

            const std::size_t maximumStartIndex = (path->size() - 2) / 2;
            std::uniform_int_distribution<std::size_t> chooseStartIndex(0, maximumStartIndex);
            const std::size_t startIndex = chooseStartIndex(random);
            if (distanceAlong(*path, startIndex) < 700.0f) {
                continue;
            }

            route.assign(path->begin() + static_cast<std::ptrdiff_t>(startIndex), path->end());
            if (route.size() < 3 || routeLength(route) < 700.0f) {
                continue;
            }
            if (type == RoadUserType::Pedestrian) {
                truncateSidewalkRouteBeforeIntersection(world, route);
                if (route.size() < 2 || routeLength(route) < 180.0f) {
                    continue;
                }
            }
        }

        const Vector2 start = route.front();
        const bool tooClose = std::any_of(
            occupiedPositions.begin(),
            occupiedPositions.end(),
            [&](Vector2 occupied) {
                return Vector2Distance(start, occupied) < MINIMUM_SPAWN_SEPARATION;
            }
        );
        if (tooClose) {
            continue;
        }

        Agent agent(start, headingOf(route[0], route[1]), type, color, useCrosswalk);
        agent.route = std::move(route);
        agent.controller.reacquireWaypoint(agent.vehicle, agent.route);
        agent.vehicle.setSpeed(type == RoadUserType::Bicycle ? 85.0f : 22.0f);
        agents.push_back(std::move(agent));
        occupiedPositions.push_back(start);
        return true;
    }

    return false;
}

void NpcRoadUsers::prepareForStep(float dt) {
    for (Agent& agent : agents) {
        if (agent.turnaroundPause > 0.0f) {
            agent.turnaroundPause = std::max(0.0f, agent.turnaroundPause - dt);
            agent.vehicle.setSpeed(0.0f);
            agent.vehicle.setSteeringAngle(0.0f);
            continue;
        }

        if (agent.route.size() >= 2 &&
            Vector2Distance(positionOf(agent.vehicle), agent.route.back()) <= TURNAROUND_DISTANCE) {
            std::reverse(agent.route.begin(), agent.route.end());
            agent.vehicle.setHeading(headingOf(agent.route[0], agent.route[1]));
            agent.vehicle.setSpeed(0.0f);
            agent.controller.reacquireWaypoint(agent.vehicle, agent.route);
            agent.turnaroundPause = agent.type == RoadUserType::Bicycle
                ? BICYCLE_TURNAROUND_PAUSE
                : (agent.usesCrosswalk
                    ? CROSSWALK_TURNAROUND_PAUSE
                    : PEDESTRIAN_TURNAROUND_PAUSE);
            continue;
        }

        agent.controller.update(agent.vehicle, agent.route, dt);
    }
}

std::vector<Vehicle*> NpcRoadUsers::getVehicles() {
    std::vector<Vehicle*> vehicles;
    vehicles.reserve(agents.size());
    for (Agent& agent : agents) {
        vehicles.push_back(&agent.vehicle);
    }
    return vehicles;
}

void NpcRoadUsers::Draw() const {
    for (const Agent& agent : agents) {
        const Vector2 center = positionOf(agent.vehicle);
        const Vector2 forward = { std::cos(agent.vehicle.getHeading()), std::sin(agent.vehicle.getHeading()) };
        if (agent.type == RoadUserType::Bicycle) {
            const Vector2 frontWheel = Vector2Add(center, Vector2Scale(forward, BICYCLE_LENGTH * 0.34f));
            const Vector2 rearWheel = Vector2Subtract(center, Vector2Scale(forward, BICYCLE_LENGTH * 0.34f));
            DrawCircleV(frontWheel, 6.0f, BLACK);
            DrawCircleV(rearWheel, 6.0f, BLACK);
            DrawLineEx(rearWheel, center, 3.0f, agent.color);
            DrawLineEx(center, frontWheel, 3.0f, agent.color);
            DrawCircleV(center, 4.0f, agent.color);
        }
        else {
            DrawCircleV(center, 7.0f, agent.color);
            DrawCircleV(Vector2Add(center, Vector2Scale(forward, 5.0f)), 3.0f, { 255, 225, 190, 255 });
        }
    }
}

std::size_t NpcRoadUsers::bicycleCount() const {
    return static_cast<std::size_t>(std::count_if(agents.begin(), agents.end(), [](const Agent& agent) {
        return agent.type == RoadUserType::Bicycle;
    }));
}

std::size_t NpcRoadUsers::pedestrianCount() const {
    return static_cast<std::size_t>(std::count_if(agents.begin(), agents.end(), [](const Agent& agent) {
        return agent.type == RoadUserType::Pedestrian;
    }));
}
