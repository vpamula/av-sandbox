#include "raylib.h"
#include "vehicle.h"
#include "road.h"
#include "route.h"
#include "path.h"
#include "lanefollower.h"
#include "trajectorygenerator.h"
#include "pathbuilder.h"
#include "world.h"

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

    std::vector<Road>& roads = world.getRoads();
    Route route;

    route.addLane(&roads[0].getRightLane());
    route.addLane(&roads[1].getRightLane());
    route.addLane(&roads[3].getRightLane());

    Path path = PathBuilder::buildRoute(route);

    Vehicle vehicle(
        0.0f,
        roads[0].getRightLane().getCenterY(),
        VEHICLE_LENGTH,
        VEHICLE_WIDTH
    );

    LaneFollower controller;
    controller.reacquireWaypoint(vehicle, path.getWaypoints());
    Camera2D camera = { 0 };
    camera.offset = (Vector2) {SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2};
    camera.zoom = 1.0f;
    bool autonomousMode = true;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (IsKeyPressed(KEY_TAB)) {
            autonomousMode =
                !autonomousMode;
            if (autonomousMode) {
                controller.reacquireWaypoint(
                    vehicle,
                    path.getWaypoints()
                );
            }
        }
        if (autonomousMode) {
            controller.Update(
                vehicle,
                path.getWaypoints(),
                dt
            );
        }
        else {
            vehicle.setSpeed(200.0f);
        }

        vehicle.Update(dt);

        camera.target = (Vector2) {
            vehicle.getX(),
            vehicle.getY()
        };

        BeginDrawing();
        ClearBackground(DARKGREEN);
        BeginMode2D(camera);
        world.Draw();
        path.Draw();
        vehicle.Draw();
        EndMode2D();
        DrawText("AV Sandbox", 20, 20, 30, WHITE);

        if (autonomousMode) {
            DrawText("AUTO", 20, 60, 30, GREEN);
        }
        else {
            DrawText("MANUAL", 20, 60, 30, RED);
        }

        DrawText(TextFormat("Speed: %.1f",vehicle.getSpeed()), 20, 100, 30, WHITE);
        DrawText(TextFormat("Heading Angle: %.2f", vehicle.getHeading() * RAD2DEG), 20, 140, 30, WHITE);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}