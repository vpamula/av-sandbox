#pragma once

#include "raylib.h"

#include <limits>

class Vehicle {
public:
    Vehicle(float startX, float startY, float length, float width, float startHeading = 0.0f);

    void Update(
        float dt,
        bool manualControl,
        float maximumSpeed = std::numeric_limits<float>::infinity()
    );
    void Draw() const;
    void DrawEgo() const;
    void Draw(Color bodyColor) const;

    float getX() const { return x; }
    float getY() const { return y; }
    float getHeading() const { return heading; }
    float getSpeed() const { return speed; }
    float getLength() const { return length; }
    float getWidth() const { return width; }

    void setSteeringAngle(float angle) { steeringAngle = angle; }
    void setSpeed(float newSpeed) { speed = newSpeed; }
    void setHeading(float newHeading) { heading = newHeading; }
    void setPosition(Vector2 position) { x = position.x; y = position.y; }
    void approachSpeed(float targetSpeed, float acceleration, float deceleration, float dt);

private:
    float x;
    float y;
    float heading;
    float speed;
    float steeringAngle;
    float length;
    float width;
};
