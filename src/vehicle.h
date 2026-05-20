#pragma once

#include "raylib.h"

class Vehicle

{

public:

    Vehicle(float startX, float startY, float length, float width);

    void Update(float dt);

    void Draw();

    float getX() {
        return x;
    }
    
    float getY() {
        return y;
    }

    float getHeading() {
        return heading;
    }

    void setSteeringAngle(float angle) {
        steeringAngle = angle;
    }

    float getSteeringAngle() {
        return steeringAngle;
    }

    void setSpeed(float sp) {
        speed = sp;
    }

    float getSpeed() {
        return speed;
    }

    void setTargetSpeed(float ts) {
        targetSpeed = ts;
    }

private:

    float x;

    float y;

    float heading;

    float speed;

    float steeringAngle;

    float vehicleLength;

    float vehicleWidth;

    float targetSpeed;

};