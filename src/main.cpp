#include "raylib.h"
#include "vehicle.h"
#include "road.h"
#include "path.h"
#include "lanefollower.h"
#include "trajectorygenerator.h"
#include <iostream>
#include <algorithm>

int main()

{
    const int screenWidth = 1920;
    const int screenHeight = 930;
    InitWindow(screenWidth, screenHeight, "AV Sandbox");
    SetTargetFPS(60);

    float vehicleLength = 80.0f;
    float vehicleWidth = 40.0f;
    float laneWidth = vehicleWidth * 2.5f;
    float roadWidth = laneWidth * 2.0f;

    Road road(
        0.0f,     // center_x
        0.0f,     // center_y
        0.0f,     // heading
        roadWidth,    // width
        2000.0f  // length
    );

    Road verticalRoad(
        1000.0f,  // center_x
        2400.0f,  // center_y
        PI / 2,   // heading
        roadWidth, // width
        5000.0f   // length 
    );

    Vehicle vehicle(
        0.0f, // vehicle_center_x
        road.getRightLane().getCenterY(), // vehicle_center_y
        vehicleLength, // vehicle_length
        vehicleWidth // vehicle_width
    );
    
    Path path;
    Lane& horizontalLane = road.getRightLane();
    Lane& verticalLane = verticalRoad.getRightLane();
    Vector2 horizontalEnd = horizontalLane.getEndPoint();
    Vector2 verticalStart = verticalLane.getStartPoint();

    Vector2 horizontalTrim = {horizontalEnd.x - 100, horizontalEnd.y};
    Vector2 verticalTrim = {verticalStart.x, verticalStart.y + 200};


    std::vector<Vector2> horizontalWaypoints = horizontalLane.generateWaypointsBetween(horizontalLane.getStartPoint(), horizontalTrim);
    std::vector<Vector2> verticalWaypoints = verticalLane.generateWaypointsBetween(verticalTrim, verticalLane.getEndPoint());
    std::vector<Vector2> turnWaypoints = TrajectoryGenerator::generateConnector(horizontalTrim, horizontalLane.getDirection(), verticalTrim, verticalLane.getDirection(), 20.0f, 5);

    for (Vector2 wp : horizontalWaypoints) {
        path.addWaypoint(wp);
    }

    for (int i = 0; i < turnWaypoints.size(); i++) {
        path.addWaypoint(turnWaypoints[i]);
    }

    for (int i = 0; i < verticalWaypoints.size(); i++) {
        path.addWaypoint(verticalWaypoints[i]);
    }

    LaneFollower controller;
    controller.reacquireWaypoint(vehicle, path.getWaypoints());
 
    Camera2D camera = { 0 };
    camera.offset = (Vector2) {screenWidth / 2, screenHeight / 2};
    camera.zoom = 1.0f;

    bool autonomousMode = true;
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (autonomousMode) {
            controller.Update(vehicle, path.getWaypoints(), dt); 
        }
        else {
            float targetSpeed = 200.0f;
            vehicle.setSpeed(targetSpeed);
        }

        if (IsKeyPressed(KEY_TAB)) {
            autonomousMode = !autonomousMode;
            if (autonomousMode) {
                controller.reacquireWaypoint(vehicle, path.getWaypoints());
            }
        }
            
        vehicle.Update(dt);
        camera.target = (Vector2) {vehicle.getX(), vehicle.getY()};
        BeginDrawing();
        ClearBackground(DARKGREEN);
            BeginMode2D(camera);
                road.Draw();
                verticalRoad.Draw();
                path.Draw();
                vehicle.Draw();
            EndMode2D();
            DrawText("AV Sandbox", 20, 20, 30, WHITE);
            if (autonomousMode) {
                DrawText("AUTO", 20, 60, 30, GREEN);
            } else {
                DrawText("MANUAL", 20, 60, 30, RED);
            }
            DrawText(TextFormat("Speed: %.1f", vehicle.getSpeed()), 20, 100, 30, WHITE);
            DrawText(TextFormat("Heading Angle: %.2f", vehicle.getHeading() * RAD2DEG), 20, 140, 30, WHITE);
            // DrawText(TextFormat("Steering Angle: %.2f", vehicle.getSteeringAngle()), 20, 180, 30, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;

}