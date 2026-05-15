#include "raylib.h"

#include "vehicle.h"

int main()

{

    const int screenWidth = 1200;

    const int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "AV Sandbox");

    SetTargetFPS(60);

    Vehicle vehicle(400, 400);

    while (!WindowShouldClose())

    {

        float dt = GetFrameTime();

        vehicle.Update(dt);

        BeginDrawing();

        ClearBackground(DARKGRAY);

        vehicle.Draw();

        DrawText("AV Sandbox", 20, 20, 30, WHITE);

        EndDrawing();

    }

    CloseWindow();

    return 0;

}