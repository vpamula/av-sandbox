#include "raylib.h"
#include "raymath.h"
#include "vehicle.h"
#include "road.h"
#include "lanefollower.h"
#include "world.h"
#include <algorithm>
#include <cmath>
#include <vector>

int main()
{
    const int SCREEN_WIDTH = 1920;
    const int SCREEN_HEIGHT = 930;
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "AV Sandbox");

    SetTargetFPS(60);

    const float VEHICLE_LENGTH = 80.0f;
    const float VEHICLE_WIDTH = 40.0f;

    const float LANE_WIDTH = 100.0f;
    const float ROAD_WIDTH = 2.0f * LANE_WIDTH;

    const float ROAD_LENGTH_LONG = 5000.0f;
    const float ROAD_LENGTH_MEDIUM = 2000.0f;
    const float ROAD_LENGTH_SHORT = 1200.0f;

    World world;

    world.addRoad(
        Road(
            0.0f,
            0.0f,
            0.0f,
            ROAD_WIDTH,
            ROAD_LENGTH_LONG
        )
    );

    world.addRoad(
        Road(
            1000.0f,
            400.0f,
            PI / 2,
            ROAD_WIDTH,
            ROAD_LENGTH_MEDIUM
        )
    );

    world.addRoad(
        Road(
            2400.0f,
            500.0f,
            PI / 2,
            ROAD_WIDTH,
            ROAD_LENGTH_SHORT
        )
    );

    world.addRoad(
        Road(
            0.0f,
            1000.0f,
            0.0f,
            ROAD_WIDTH,
            ROAD_LENGTH_LONG
        )
    );

    world.computeIntersections();

    const std::vector<Road>& roads = world.getRoads();
    Vehicle vehicle(
        0.0f,
        roads[0].getRightLane().getCenter().y,
        VEHICLE_LENGTH,
        VEHICLE_WIDTH
    );

    LaneFollower controller;
    Camera2D camera = { 0 };
    camera.offset = (Vector2) {SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2};
    camera.zoom = 1.0f;

    bool autonomousMode = true;
    bool routeSelected = false;
    bool paused = true;
    bool panningCamera = false;
    bool followVehicleCamera = true;
    bool leftDragPan = false;
    Vector2 leftDragStart = {0.0f, 0.0f};
    Vector2 startRoutePoint = {0.0f, 0.0f};
    std::vector<Vector2> routeToSelectedStart;
    std::vector<Vector2> routeFromSelectedStart;
    bool drivingToSelectedStart = false;
    bool routeCompleted = false;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
            if (!paused) {
                followVehicleCamera = true;
            }
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
            autonomousMode = true;
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON) || IsMouseButtonPressed(MOUSE_MIDDLE_BUTTON)) {
            panningCamera = true;
            followVehicleCamera = false;
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
                    routeSelected = true;
                    routeToSelectedStart.clear();
                    routeFromSelectedStart.clear();
                    drivingToSelectedStart = false;
                    routeCompleted = false;
                }
                else {
                    const Vector2 endRoutePoint = snappedPoint.value();
                    const Vector2 vehiclePosition = { vehicle.getX(), vehicle.getY() };
                    routeToSelectedStart = world.buildRouteBetweenPoints(vehiclePosition, startRoutePoint);
                    routeFromSelectedStart = world.buildRouteBetweenPoints(startRoutePoint, endRoutePoint);

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
                }
            }

            leftDragPan = false;
            panningCamera = false;
            if (!paused) {
                followVehicleCamera = true;
            }
            continue;
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
            leftDragPan = false;
            if (!IsMouseButtonDown(MOUSE_RIGHT_BUTTON) && !IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
                panningCamera = false;
            }
            if (!paused && !panningCamera) {
                followVehicleCamera = true;
            }
        }

        auto zoomAroundCursor = [&](float zoomFactor) {
            Vector2 mouseScreen = GetMousePosition();
            Vector2 worldBeforeZoom = GetScreenToWorld2D(mouseScreen, camera);
            float newZoom = std::clamp(camera.zoom * zoomFactor, 0.3f, 3.0f);
            camera.zoom = newZoom;
            followVehicleCamera = false;

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

        if (!paused) {
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

            if (!routeCompleted) {
                vehicle.Update(dt, !autonomousMode);
            }
        }
        else {
            vehicle.setSpeed(0.0f);
        }

        if (!panningCamera && followVehicleCamera) {
            camera.target = (Vector2) {
                vehicle.getX(),
                vehicle.getY()
            };
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
        const char* motionStatus = routeCompleted ? "ARRIVED" : (paused ? "PAUSED" : "RUNNING");
        const Color motionStatusColor = routeCompleted ? SKYBLUE : (paused ? YELLOW : GREEN);
        DrawText(motionStatus, 20, 180, 30, motionStatusColor);
        DrawText("BLUE: current position to start click", 20, 220, 18, SKYBLUE);
        DrawText("YELLOW: start click to destination click", 20, 245, 18, YELLOW);

        const int bottomLeftX = 20;
        const int bottomLeftY = SCREEN_HEIGHT - 150;

        DrawText("LEFT CLICK = set start point", bottomLeftX, bottomLeftY, 20, WHITE);
        DrawText("SECOND LEFT CLICK = set end point", bottomLeftX, bottomLeftY + 30, 20, WHITE);
        DrawText("RIGHT DRAG pans camera", bottomLeftX, bottomLeftY + 60, 20, WHITE);
        DrawText("SPACE toggles pause", bottomLeftX, bottomLeftY + 90, 20, WHITE);
        DrawText("MOUSE WHEEL or +/- zooms camera", bottomLeftX, bottomLeftY + 120, 20, WHITE);
        DrawText("R resets route", bottomLeftX, bottomLeftY + 150, 20, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
