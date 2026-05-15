#pragma once

#include "raylib.h"

class Vehicle

{

public:

    Vehicle(float startX, float startY);

    void Update(float dt);

    void Draw();

private:

    float x;

    float y;

    float heading;

    float speed;

    float steeringAngle;

};