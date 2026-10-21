#pragma once

#include "raylib.h"

class Vehicle {
public:
    Vehicle(float startX, float startY, float length, float width);

    void Update(float dt, bool manualControl);
    void Draw() const;

    float getX() const { return x; }
    float getY() const { return y; }
    float getHeading() const { return heading; }
    float getSpeed() const { return speed; }
    float getLength() const { return length; }

    void setSteeringAngle(float angle) { steeringAngle = angle; }
    void setSpeed(float newSpeed) { speed = newSpeed; }

private:
    float x;
    float y;
    float heading;
    float speed;
    float steeringAngle;
    float length;
    float width;
};
