#include "raylib.h"
#include "vehicle.h"
#include "road.h"

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

    Camera2D camera = { 0 };
    camera.offset = (Vector2) {screenWidth / 2, screenHeight / 2};
    camera.zoom = 1.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        vehicle.Update(dt);
        camera.target = (Vector2) {vehicle.getX(), vehicle.getY()};
        BeginDrawing();
        ClearBackground(DARKGREEN);
            BeginMode2D(camera);
                road.Draw();
                vehicle.Draw();
            EndMode2D();
            DrawText("AV Sandbox", 20, 20, 30, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;

}