#include "vehicle.h"

#include <cmath>

Vehicle::Vehicle(float startX, float startY)

{   
    vehicleLength = 80.0f;
    vehicleWidth = 40.0f;
    x = startX;
    y = startY;
    heading = 0.0f;
    speed = 0.0f;
    steeringAngle = 0.0f;
}

void Vehicle::Update(float dt)

{
    // Throttle
    if (IsKeyDown(KEY_W)) {
        speed += 200.0f * dt;
    }

    if (IsKeyDown(KEY_S)){
        speed -= 200.0f * dt;
    }

    // Steering
    if (IsKeyDown(KEY_A)) {
        steeringAngle -= 1.0f * dt;
    }

    else if (IsKeyDown(KEY_D)) {
        steeringAngle += 1.0f * dt;
    }

    else {
        steeringAngle += -8.0f * steeringAngle * dt;
    }

    // Clamp steering
    if (steeringAngle > 0.87f)
        steeringAngle = 0.87f;

    if (steeringAngle < -0.87f)
        steeringAngle = -0.87f;

    // Friction
    speed *= 0.99f;

    // Vehicle motion
    heading += (speed / (vehicleLength * (1.0f + 0.00001 * speed * speed))) * tan(steeringAngle) * dt;    
    x += speed * cos(heading) * dt;
    y += speed * sin(heading) * dt;

}

void Vehicle::Draw()

{
    Rectangle rect = { x , y , vehicleLength, vehicleWidth};
    Vector2 origin = { vehicleLength / 2, vehicleWidth / 2 };
    float frontOffset = vehicleLength / 2.5;
    float x_front = x + frontOffset * cos(heading);
    float y_front = y + frontOffset * sin(heading);
    DrawRectanglePro(
        rect,
        origin,
        heading * RAD2DEG,
        RED
    );
    DrawCircleV(
        (Vector2){ x_front, y_front },
        8,
        YELLOW
    );
}