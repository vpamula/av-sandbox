#include "raylib.h"
#include "raymath.h"
#include "vehicle.h"
#include "road.h"
#include "lanefollower.h"
#include "npctraffic.h"
#include "npcroadusers.h"
#include "perception.h"
#include "trafficsafety.h"
#include "stopsigncoordinator.h"
#include "world.h"
#include <algorithm>
#include <cmath>
#include <vector>

int main()
{
    const int SCREEN_WIDTH = 1920;
    const int SCREEN_HEIGHT = 930;
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "AV Sandbox - 5x5 Downtown Grid");

    SetTargetFPS(60);

    const float VEHICLE_LENGTH = 80.0f;
    const float VEHICLE_WIDTH = 40.0f;

    World world = World::makeDowntownGrid();
    const std::vector<Road>& roads = world.getRoads();
    const std::vector<Vector2>& startingLane = roads[2].getRightLane().getWaypoints();
    const Vector2 vehicleStart = startingLane[startingLane.size() / 2];
    Vehicle vehicle(
        vehicleStart.x,
        vehicleStart.y,
        VEHICLE_LENGTH,
        VEHICLE_WIDTH
    );

    LaneFollower controller;
    NpcTraffic npcTraffic(world, vehicle, 12);
    NpcRoadUsers npcRoadUsers(world, vehicle, npcTraffic.getVehicles(), 4, 6);
    Perception perception;
    TrafficSafety trafficSafety;
    StopSignCoordinator stopSignCoordinator;
    Camera2D camera = { 0 };
    camera.offset = (Vector2) {SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2};
    camera.zoom = 0.31f;

    bool autonomousMode = true;
    bool routeSelected = false;
    bool paused = true;
    bool panningCamera = false;
    bool leftDragPan = false;
    Vector2 leftDragStart = {0.0f, 0.0f};
    std::vector<Vector2> selectedRoutePoints;
    std::vector<Vector2> routeToSelectedStart;
    std::vector<Vector2> routeFromSelectedStart;
    bool drivingToSelectedStart = false;
    bool routeCompleted = false;

    auto buildRouteThroughPoints = [&world](std::vector<Vector2>& points) {
        std::vector<Vector2> route;
        if (points.size() < 2) {
            return route;
        }

        for (std::size_t index = 1; index < points.size(); ++index) {
            const Vector2 segmentStart = route.empty() ? points[index - 1] : route.back();
            std::vector<Vector2> segment = world.buildRouteBetweenPoints(
                segmentStart,
                points[index]
            );
            if (segment.empty()) {
                return std::vector<Vector2>{};
            }

            points[index - 1] = segment.front();
            points[index] = segment.back();

            std::size_t firstPoint = 0;
            if (!route.empty() && Vector2Distance(route.back(), segment.front()) <= 1.0f) {
                firstPoint = 1;
            }
            route.insert(
                route.end(),
                segment.begin() + static_cast<std::ptrdiff_t>(firstPoint),
                segment.end()
            );
        }
        return route;
    };

    auto updatePerception = [&]() {
        const std::vector<Vehicle*> trafficVehicles = npcTraffic.getVehicles();
        const std::vector<Vehicle*> roadUserVehicles = npcRoadUsers.getVehicles();
        std::vector<PerceptionTarget> targets;
        targets.reserve(trafficVehicles.size() + roadUserVehicles.size());
        for (std::size_t index = 0; index < trafficVehicles.size(); ++index) {
            const Vehicle& target = *trafficVehicles[index];
            targets.push_back({
                static_cast<int>(index),
                PerceivedAgentType::Vehicle,
                { target.getX(), target.getY() },
                target.getHeading(),
                target.getSpeed(),
                target.getLength(),
                target.getWidth()
            });
        }
        for (std::size_t index = 0; index < roadUserVehicles.size(); ++index) {
            const Vehicle& target = *roadUserVehicles[index];
            targets.push_back({
                static_cast<int>(trafficVehicles.size() + index),
                index < npcRoadUsers.bicycleCount()
                    ? PerceivedAgentType::Bicycle
                    : PerceivedAgentType::Pedestrian,
                { target.getX(), target.getY() },
                target.getHeading(),
                target.getSpeed(),
                target.getLength(),
                target.getWidth()
            });
        }
        perception.update(vehicle, targets, world);
    };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        if (IsKeyPressed(KEY_TAB)) {
            autonomousMode = !autonomousMode;
            if (autonomousMode && !routeToSelectedStart.empty() && !routeFromSelectedStart.empty()) {
                const std::vector<Vector2>& activeRoute = drivingToSelectedStart
                    ? routeToSelectedStart
                    : routeFromSelectedStart;
                controller.reacquireWaypoint(vehicle, activeRoute);
            }
        }

        if (IsKeyPressed(KEY_R)) {
            routeSelected = false;
            selectedRoutePoints.clear();
            routeToSelectedStart.clear();
            routeFromSelectedStart.clear();
            drivingToSelectedStart = false;
            routeCompleted = false;
            controller.setRouteControls({});
            autonomousMode = true;
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON) || IsMouseButtonPressed(MOUSE_MIDDLE_BUTTON)) {
            panningCamera = true;
        }

        if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON) || IsMouseButtonReleased(MOUSE_MIDDLE_BUTTON)) {
            panningCamera = false;
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            leftDragPan = true;
            leftDragStart = GetMousePosition();
            panningCamera = false;
        }

        if (leftDragPan && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            Vector2 dragDelta = { GetMousePosition().x - leftDragStart.x, GetMousePosition().y - leftDragStart.y };
            float dragDistance = std::hypot(dragDelta.x, dragDelta.y);
            if (dragDistance > 8.0f) {
                panningCamera = true;
            }
            else {
                panningCamera = false;
            }
        }

        if (panningCamera) {
            Vector2 mouseDelta = GetMouseDelta();
            camera.target.x -= mouseDelta.x / camera.zoom;
            camera.target.y -= mouseDelta.y / camera.zoom;
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON) && leftDragPan && !panningCamera) {
            Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), camera);
            auto snappedPoint = world.findNearestRoadPoint(mouseWorld, 200.0f);

            if (snappedPoint.has_value()) {
                if (selectedRoutePoints.empty()) {
                    selectedRoutePoints.push_back(snappedPoint.value());
                    routeSelected = true;
                    routeToSelectedStart.clear();
                    routeFromSelectedStart.clear();
                    drivingToSelectedStart = false;
                    routeCompleted = false;
                    controller.setRouteControls({});
                }
                else {
                    std::vector<Vector2> candidatePoints = selectedRoutePoints;
                    candidatePoints.push_back(snappedPoint.value());
                    std::vector<Vector2> candidateRoute = buildRouteThroughPoints(candidatePoints);
                    if (!candidateRoute.empty()) {
                        const bool creatingFirstSegment = selectedRoutePoints.size() == 1;
                        selectedRoutePoints = std::move(candidatePoints);
                        routeFromSelectedStart = std::move(candidateRoute);

                        if (creatingFirstSegment) {
                            const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
                            routeToSelectedStart = world.buildRouteBetweenPoints(
                                vehiclePosition,
                                routeFromSelectedStart.front()
                            );
                            if (routeToSelectedStart.empty()) {
                                routeToSelectedStart.push_back(vehiclePosition);
                            }
                            // Keep the vehicle's exact current position as the
                            // visible and followed start of the approach leg.
                            else if (Vector2Distance(vehiclePosition, routeToSelectedStart.front()) > 1.0f) {
                                routeToSelectedStart.insert(routeToSelectedStart.begin(), vehiclePosition);
                            }
                            routeSelected = false;
                            drivingToSelectedStart =
                                Vector2Distance(vehiclePosition, routeFromSelectedStart.front()) > 60.0f;
                        }

                        if (!routeToSelectedStart.empty()) {
                            routeCompleted = false;
                            const std::vector<Vector2>& activeRoute = drivingToSelectedStart
                                ? routeToSelectedStart
                                : routeFromSelectedStart;
                            controller.setRouteControls(
                                world.getRouteControlsAlongRoute(activeRoute, vehicle.getLength())
                            );
                            controller.reacquireWaypoint(vehicle, activeRoute);
                            autonomousMode = true;
                        }
                    }
                }
            }

            leftDragPan = false;
            panningCamera = false;
            continue;
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            leftDragPan = false;
            if (!IsMouseButtonDown(MOUSE_RIGHT_BUTTON) && !IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
                panningCamera = false;
            }
        }

        auto zoomAroundCursor = [&](float zoomFactor) {
            Vector2 mouseScreen = GetMousePosition();
            Vector2 worldBeforeZoom = GetScreenToWorld2D(mouseScreen, camera);
            float newZoom = std::clamp(camera.zoom * zoomFactor, 0.3f, 3.0f);
            camera.zoom = newZoom;

            Vector2 cursorOffset = Vector2Subtract(mouseScreen, camera.offset);
            Vector2 scaledOffset = Vector2Scale(cursorOffset, 1.0f / camera.zoom);
            camera.target = Vector2Subtract(worldBeforeZoom, scaledOffset);
        };

        float wheelMove = GetMouseWheelMove();
        if (wheelMove != 0.0f) {
            zoomAroundCursor(1.0f + wheelMove * 0.15f);
        }

        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
            zoomAroundCursor(1.15f);
        }
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
            zoomAroundCursor(1.0f / 1.15f);
        }

        // The simulated sensor observes the current scene before the ego
        // controller chooses its speed for this frame.
        updatePerception();

        if (!paused) {
            world.updateTrafficLights(dt);
            controller.setTrafficLightStates(world.getTrafficLightStates());

            std::vector<StopSignParticipant> stopSignParticipants = npcTraffic.getStopSignParticipants();
            stopSignParticipants.push_back({ &vehicle, &controller });
            stopSignCoordinator.update(world, stopSignParticipants);

            if (routeCompleted) {
                vehicle.setSpeed(0.0f);
                vehicle.setSteeringAngle(0.0f);
            }
            else if (autonomousMode && !routeToSelectedStart.empty() && !routeFromSelectedStart.empty()) {
                const std::vector<Vector2>& activeRoute = drivingToSelectedStart
                    ? routeToSelectedStart
                    : routeFromSelectedStart;
                controller.update(vehicle, activeRoute, dt);
                if (drivingToSelectedStart && !routeToSelectedStart.empty() &&
                    Vector2Distance(
                        { vehicle.getX(), vehicle.getY() },
                        routeToSelectedStart.back()
                    ) <= 60.0f) {
                    drivingToSelectedStart = false;
                    controller.setRouteControls(
                        world.getRouteControlsAlongRoute(routeFromSelectedStart, vehicle.getLength())
                    );
                    controller.reacquireWaypoint(vehicle, routeFromSelectedStart);
                }

                constexpr float ARRIVAL_RADIUS = 40.0f;
                if (!drivingToSelectedStart && !routeFromSelectedStart.empty() &&
                    Vector2Distance(
                        { vehicle.getX(), vehicle.getY() },
                        routeFromSelectedStart.back()
                    ) <= ARRIVAL_RADIUS) {
                    routeCompleted = true;
                    vehicle.setSpeed(0.0f);
                    vehicle.setSteeringAngle(0.0f);
                }
            }
            else {
                if (autonomousMode) {
                    vehicle.setSpeed(200.0f);
                }
            }

            if (autonomousMode && !routeCompleted) {
                const float perceptionSpeedLimit = perception.getRecommendedEgoSpeed(vehicle);
                if (perceptionSpeedLimit < vehicle.getSpeed()) {
                    vehicle.approachSpeed(perceptionSpeedLimit, 180.0f, 700.0f, dt);
                }
            }

            npcTraffic.prepareForStep(world, vehicle, dt);
            std::vector<Vehicle*> crossingTraffic = { &vehicle };
            std::vector<Vehicle*> crossingNpcCars = npcTraffic.getVehicles();
            crossingTraffic.insert(
                crossingTraffic.end(),
                crossingNpcCars.begin(),
                crossingNpcCars.end()
            );
            std::vector<Vehicle*> crossingBicycles = npcRoadUsers.getBicycleVehicles();
            crossingTraffic.insert(
                crossingTraffic.end(),
                crossingBicycles.begin(),
                crossingBicycles.end()
            );
            npcRoadUsers.prepareForStep(crossingTraffic, dt);
            std::vector<Vehicle*> vehicles = { &vehicle };
            std::vector<Vehicle*> npcVehicles = npcTraffic.getVehicles();
            vehicles.insert(vehicles.end(), npcVehicles.begin(), npcVehicles.end());
            const std::size_t roadUserPriorityStart = vehicles.size();
            std::vector<Vehicle*> roadUserVehicles = npcRoadUsers.getVehicles();
            const std::size_t pedestrianPriorityStart =
                roadUserPriorityStart + npcRoadUsers.bicycleCount();
            vehicles.insert(vehicles.end(), roadUserVehicles.begin(), roadUserVehicles.end());
            trafficSafety.step(
                world,
                vehicles,
                !autonomousMode && !routeCompleted,
                dt,
                roadUserPriorityStart,
                pedestrianPriorityStart
            );
            npcRoadUsers.constrainPedestriansToRoutes();
        }
        else {
            vehicle.setSpeed(0.0f);
        }

        BeginDrawing();
        ClearBackground(DARKGREEN);
        BeginMode2D(camera);
        world.Draw();

        if (routeSelected) {
            DrawCircleV(selectedRoutePoints.front(), 12.0f, GREEN);
        }

        if (!routeToSelectedStart.empty() && !routeFromSelectedStart.empty()) {
            if (routeToSelectedStart.size() > 1) {
                DrawLineStrip(routeToSelectedStart.data(), routeToSelectedStart.size(), SKYBLUE);
            }
            if (routeFromSelectedStart.size() > 1) {
                DrawLineStrip(routeFromSelectedStart.data(), routeFromSelectedStart.size(), YELLOW);
            }

            DrawCircleV(routeToSelectedStart.front(), 8.0f, SKYBLUE);
            DrawCircleV(routeToSelectedStart.back(), 10.0f, GREEN);
            for (std::size_t index = 0; index < selectedRoutePoints.size(); ++index) {
                const Color pointColor = index == 0
                    ? GREEN
                    : (index + 1 == selectedRoutePoints.size() ? RED : ORANGE);
                DrawCircleV(selectedRoutePoints[index], 10.0f, pointColor);
            }
        }

        npcTraffic.Draw();
        npcRoadUsers.Draw();
        perception.Draw(vehicle);
        vehicle.DrawEgo();
        EndMode2D();

        DrawText("AV Sandbox", 20, 20, 30, WHITE);

        if (autonomousMode) {
            DrawText("AUTO", 20, 60, 30, GREEN);
        }
        else {
            DrawText("MANUAL", 20, 60, 30, RED);
        }

        DrawText(TextFormat("Speed: %.1f", vehicle.getSpeed()), 20, 100, 30, WHITE);
        DrawText(TextFormat("Heading Angle: %.2f", vehicle.getHeading() * RAD2DEG), 20, 140, 30, WHITE);
        const char* motionStatus = routeCompleted
            ? "ARRIVED"
            : (paused
                ? "PAUSED"
                : (controller.isWaitingAtStopSign()
                    ? "STOP SIGN"
                    : (controller.isWaitingAtTrafficLight() ? "TRAFFIC LIGHT" : "RUNNING")));
        const Color motionStatusColor = routeCompleted
            ? SKYBLUE
            : (paused
                ? YELLOW
                : ((controller.isWaitingAtStopSign() || controller.isWaitingAtTrafficLight())
                    ? ORANGE
                    : GREEN));
        DrawText(motionStatus, 20, 180, 30, motionStatusColor);
        DrawText(
            TextFormat(
                "NPCs: %i CARS / %i BIKES / %i PEDS",
                static_cast<int>(npcTraffic.size()),
                static_cast<int>(npcRoadUsers.bicycleCount()),
                static_cast<int>(npcRoadUsers.pedestrianCount())
            ),
            20,
            220,
            18,
            WHITE
        );
        DrawText("BLUE: current position to first route point", 20, 245, 18, SKYBLUE);
        DrawText("YELLOW: route through selected points in order", 20, 270, 18, YELLOW);
        DrawText("EGO: WHITE BODY + MAGENTA STRIPE", 20, 320, 18, { 255, 35, 190, 255 });
        DrawText(
            TextFormat("PERCEPTION: %i OBJECTS", static_cast<int>(perception.getObjects().size())),
            20,
            295,
            18,
            LIME
        );

        const int bottomLeftX = 20;
        const int bottomLeftY = SCREEN_HEIGHT - 150;

        DrawText("LEFT CLICK = add a route point (keep clicking to extend)", bottomLeftX, bottomLeftY, 20, WHITE);
        DrawText("R resets the route before starting a new one", bottomLeftX, bottomLeftY + 30, 20, WHITE);
        DrawText("RIGHT / MIDDLE DRAG pans; LEFT DRAG also pans", bottomLeftX, bottomLeftY + 60, 20, WHITE);
        DrawText("SPACE toggles pause", bottomLeftX, bottomLeftY + 90, 20, WHITE);
        DrawText("MOUSE WHEEL or +/- zooms camera", bottomLeftX, bottomLeftY + 120, 20, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
