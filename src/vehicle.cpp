#include "vehicle.h"

#include <cmath>

Vehicle::Vehicle(float startX, float startY)

{

    x = startX;

    y = startY;

    heading = 0.0f;

    speed = 0.0f;

    steeringAngle = 0.0f;

}

void Vehicle::Update(float dt)

{

    // Throttle

    if (IsKeyDown(KEY_W))

    {

        speed += 200.0f * dt;

    }

    if (IsKeyDown(KEY_S))

    {

        speed -= 200.0f * dt;

    }

    // Steering

    if (IsKeyDown(KEY_A))

    {

        steeringAngle -= 2.0f * dt;

    }

    if (IsKeyDown(KEY_D))

    {

        steeringAngle += 2.0f * dt;

    }

    // Clamp steering

    if (steeringAngle > 1.0f)

        steeringAngle = 1.0f;

    if (steeringAngle < -1.0f)

        steeringAngle = -1.0f;

    // Friction

    speed *= 0.99f;

    // Vehicle motion

    heading += steeringAngle * dt;

    x += speed * cos(heading) * dt;

    y += speed * sin(heading) * dt;

}

void Vehicle::Draw()

{

    Rectangle rect = { x, y, 80, 40 };

    Vector2 origin = { 40, 20 };

    DrawRectanglePro(

        rect,

        origin,

        heading * RAD2DEG,

        RED

    );

}