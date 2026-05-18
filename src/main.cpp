#include "raylib.h"
#include "vehicle.h"
#include "road.h"
#include "path.h"
#include <iostream>
#include <algorithm>

int findForwardWaypoints(const std::vector<Vector2>& waypoints, Vehicle& vehicle) {
    for (int i = 0; i < waypoints.size(); i++) {
        if (waypoints[i].x > vehicle.getX()) {
            return std::min(i + 3, (int)waypoints.size() - 1);
        }
    }
    return waypoints.size() - 1;
}

void followLane(Vehicle& vehicle, const std::vector<Vector2>& waypoints, int& currentWaypointIndex) {
    int lookaheadIndex = std::min(currentWaypointIndex + 5, (int)waypoints.size() - 1);
    Vector2 target = waypoints[lookaheadIndex];
    float dx = target.x - vehicle.getX();
    float dy = target.y - vehicle.getY();
    float desired_heading = atan2(dy, dx);
    float headingError = desired_heading - vehicle.getHeading();
    float steeringGain = 5.0f;
    float steeringCommand = headingError * steeringGain;
    steeringCommand = std::clamp(steeringCommand, -0.4f, 0.4f);
    vehicle.setSteeringAngle(steeringCommand);
    float distance = sqrt(dx*dx + dy*dy);
    if (distance < 80.0f and currentWaypointIndex < waypoints.size() - 1) {
        currentWaypointIndex++;
    }
}
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
        10000.0f  // length
    );

    Vehicle vehicle(
        0.0f, // vehicle_center_x
        road.getRightLaneCenter(), // vehicle_center_y
        vehicleLength, // vehicle_length
        vehicleWidth // vehicle_width
    );
    
    Path path;
    std::vector<Vector2> waypoints = road.generateLaneWaypoints(true);
    for (Vector2 wp : waypoints) {
        path.addWaypoint(wp);
    }
    int currentWaypointIndex = path.getWaypoints().size() / 2;
 
    Camera2D camera = { 0 };
    camera.offset = (Vector2) {screenWidth / 2, screenHeight / 2};
    camera.zoom = 1.0f;

    bool autonomousMode = true;


    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (autonomousMode) {
            followLane(vehicle, path.getWaypoints(), currentWaypointIndex);
            }

        if (IsKeyPressed(KEY_TAB)) {
            autonomousMode = !autonomousMode;
            if (autonomousMode) {
                currentWaypointIndex = findForwardWaypoints(path.getWaypoints(), vehicle);
            }
        }
        float targetSpeed = 200.0f;
        vehicle.setSpeed(targetSpeed);
            
        vehicle.Update(dt);
        camera.target = (Vector2) {vehicle.getX(), vehicle.getY()};
        BeginDrawing();
        ClearBackground(DARKGREEN);
            BeginMode2D(camera);
                road.Draw();
                path.Draw();
                vehicle.Draw();
            EndMode2D();
            DrawText("AV Sandbox", 20, 20, 30, WHITE);
            if (autonomousMode) {
                DrawText("AUTO", 20, 60, 30, GREEN);
            } else {
                DrawText("MANUAL", 20, 60, 30, RED);
            }
        EndDrawing();
    }

    CloseWindow();

    return 0;

}