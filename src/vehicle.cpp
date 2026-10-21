#include "vehicle.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float MAX_STEERING_ANGLE = 0.87f;
constexpr float STEERING_RETURN_RATE = 8.0f;
constexpr float ROLLING_FRICTION_PER_FRAME = 0.99f;
}

Vehicle::Vehicle(float startX, float startY, float length, float width, float startHeading)
    : x(startX),
      y(startY),
      heading(startHeading),
      speed(200.0f),
      steeringAngle(0.0f),
      length(length),
      width(width) {}

void Vehicle::approachSpeed(float targetSpeed, float acceleration, float deceleration, float dt) {
    const float speedChange = targetSpeed - speed;
    const float maxChange = (speedChange >= 0.0f ? acceleration : deceleration) * dt;
    speed += std::clamp(speedChange, -maxChange, maxChange);
    speed = std::max(0.0f, speed);
}

void Vehicle::Update(float dt, bool manualControl, float maximumSpeed) {
    if (manualControl) {
        if (IsKeyDown(KEY_W)) speed += 200.0f * dt;
        if (IsKeyDown(KEY_S)) speed -= 200.0f * dt;

        if (IsKeyDown(KEY_A)) {
            steeringAngle -= dt;
        }
        else if (IsKeyDown(KEY_D)) {
            steeringAngle += dt;
        }
        else {
            steeringAngle -= STEERING_RETURN_RATE * steeringAngle * dt;
        }

        steeringAngle = std::clamp(steeringAngle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
        speed = std::max(0.0f, speed);
    }

    // Scale friction by elapsed time so motion is stable if the frame rate changes.
    speed *= std::pow(ROLLING_FRICTION_PER_FRAME, dt * 60.0f);
    speed = std::min(speed, std::max(0.0f, maximumSpeed));

    const float speedSquared = speed * speed;
    const float turnRate = speed / (length * (1.0f + 0.00001f * speedSquared));
    heading += turnRate * std::tan(steeringAngle) * dt;
    x += speed * std::cos(heading) * dt;
    y += speed * std::sin(heading) * dt;
}

void Vehicle::Draw() const {
    Draw(MAROON);
}

void Vehicle::DrawEgo() const {
    constexpr Color OUTLINE_COLOR = { 18, 24, 31, 255 };
    constexpr Color BODY_COLOR = { 240, 246, 250, 255 };
    constexpr Color ACCENT_COLOR = { 255, 35, 190, 255 };
    const float rotation = heading * RAD2DEG;

    DrawRectanglePro(
        { x, y, length + 12.0f, width + 12.0f },
        { (length + 12.0f) / 2.0f, (width + 12.0f) / 2.0f },
        rotation,
        OUTLINE_COLOR
    );
    DrawRectanglePro(
        { x, y, length, width },
        { length / 2.0f, width / 2.0f },
        rotation,
        BODY_COLOR
    );
    DrawRectanglePro(
        { x, y, length * 0.72f, width * 0.32f },
        { length * 0.36f, width * 0.16f },
        rotation,
        ACCENT_COLOR
    );

    const float frontOffset = length / 2.5f;
    DrawCircleV(
        { x + frontOffset * std::cos(heading), y + frontOffset * std::sin(heading) },
        7.0f,
        ACCENT_COLOR
    );
}

void Vehicle::Draw(Color bodyColor) const {
    DrawRectanglePro(
        { x, y, length, width },
        { length / 2.0f, width / 2.0f },
        heading * RAD2DEG,
        bodyColor
    );

    const float frontOffset = length / 2.5f;
    DrawCircleV(
        { x + frontOffset * std::cos(heading), y + frontOffset * std::sin(heading) },
        8.0f,
        WHITE
    );
}
