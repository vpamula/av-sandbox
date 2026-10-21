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
