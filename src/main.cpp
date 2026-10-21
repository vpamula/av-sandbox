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
    Vector2 startRoutePoint = {0.0f, 0.0f};
    Vector2 startRouteClickPoint = {0.0f, 0.0f};
    std::vector<Vector2> routeToSelectedStart;
    std::vector<Vector2> routeFromSelectedStart;
    bool drivingToSelectedStart = false;
    bool routeCompleted = false;

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
            routeToSelectedStart.clear();
            routeFromSelectedStart.clear();
            drivingToSelectedStart = false;
            routeCompleted = false;
            startRouteClickPoint = {0.0f, 0.0f};
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
                if (!routeSelected) {
                    startRoutePoint = snappedPoint.value();
                    startRouteClickPoint = mouseWorld;
                    routeSelected = true;
                    routeToSelectedStart.clear();
                    routeFromSelectedStart.clear();
                    drivingToSelectedStart = false;
                    routeCompleted = false;
                }
                else {
                    const Vector2 endRoutePoint = mouseWorld;
                    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
                    routeFromSelectedStart = world.buildRouteBetweenPoints(
                        startRouteClickPoint,
                        endRoutePoint
                    );
                    if (!routeFromSelectedStart.empty()) {
                        startRoutePoint = routeFromSelectedStart.front();
                        routeToSelectedStart = world.buildRouteBetweenPoints(vehiclePosition, startRoutePoint);
                    }

                    if (!routeToSelectedStart.empty() && !routeFromSelectedStart.empty()) {
                        // The graph route starts at its nearest lane node. Keep
                        // the vehicle's exact current position as the visible
                        // and followed start of the approach leg.
                        if (Vector2Distance(vehiclePosition, routeToSelectedStart.front()) > 1.0f) {
                            routeToSelectedStart.insert(routeToSelectedStart.begin(), vehiclePosition);
                        }

                        drivingToSelectedStart = Vector2Distance(vehiclePosition, startRoutePoint) > 60.0f;
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
                    else {
                        routeToSelectedStart.clear();
                        routeFromSelectedStart.clear();
                        drivingToSelectedStart = false;
                        routeCompleted = false;
                    }

                    routeSelected = false;
                    startRoutePoint = {0.0f, 0.0f};
                    startRouteClickPoint = {0.0f, 0.0f};
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
            npcRoadUsers.prepareForStep(dt);
            std::vector<Vehicle*> vehicles = { &vehicle };
            std::vector<Vehicle*> npcVehicles = npcTraffic.getVehicles();
            vehicles.insert(vehicles.end(), npcVehicles.begin(), npcVehicles.end());
            const std::size_t roadUserPriorityStart = vehicles.size();
            std::vector<Vehicle*> roadUserVehicles = npcRoadUsers.getVehicles();
            vehicles.insert(vehicles.end(), roadUserVehicles.begin(), roadUserVehicles.end());
            trafficSafety.step(
                vehicles,
                !autonomousMode && !routeCompleted,
                dt,
                roadUserPriorityStart
            );
        }
        else {
            vehicle.setSpeed(0.0f);
        }

        BeginDrawing();
        ClearBackground(DARKGREEN);
        BeginMode2D(camera);
        world.Draw();

        if (routeSelected) {
            DrawCircleV(startRoutePoint, 12.0f, GREEN);
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
            DrawCircleV(routeFromSelectedStart.back(), 10.0f, RED);
        }

        npcTraffic.Draw();
        npcRoadUsers.Draw();
        perception.Draw(vehicle);
        vehicle.Draw();
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
        DrawText("BLUE: current position to start click", 20, 245, 18, SKYBLUE);
        DrawText("YELLOW: start click to destination click", 20, 270, 18, YELLOW);
        DrawText(
            TextFormat("PERCEPTION: %i OBJECTS", static_cast<int>(perception.getObjects().size())),
            20,
            295,
            18,
            LIME
        );

        const int bottomLeftX = 20;
        const int bottomLeftY = SCREEN_HEIGHT - 150;

        DrawText("LEFT CLICK = set start point", bottomLeftX, bottomLeftY, 20, WHITE);
        DrawText("SECOND LEFT CLICK = set end point", bottomLeftX, bottomLeftY + 30, 20, WHITE);
        DrawText("RIGHT / MIDDLE DRAG pans; LEFT DRAG also pans", bottomLeftX, bottomLeftY + 60, 20, WHITE);
        DrawText("SPACE toggles pause", bottomLeftX, bottomLeftY + 90, 20, WHITE);
        DrawText("MOUSE WHEEL or +/- zooms camera", bottomLeftX, bottomLeftY + 120, 20, WHITE);
        DrawText("R resets route", bottomLeftX, bottomLeftY + 150, 20, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
