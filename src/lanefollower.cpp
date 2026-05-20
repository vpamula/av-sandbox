#include "lanefollower.h"

LaneFollower::LaneFollower() {
    currentWaypointIndex = 0;
    steeringGain = 5.0f;
    lookaheadPoints = 5;
}

void LaneFollower::reacquireWaypoint(Vehicle& vehicle, const std::vector<Vector2>& waypoints) {
    float minDistance = 1e9;
    for (int i = 0; i < waypoints.size(); i++) {
        float dx = waypoints[i].x - vehicle.getX();
        float dy = waypoints[i].y - vehicle.getY();
        float dist = sqrt(dx*dx + dy*dy);
        if (dist < minDistance) {
            minDistance = dist;
            currentWaypointIndex = std::min(i + 3, (int)waypoints.size() - 1);
        }
    }
}

void LaneFollower::Update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt) {
        int lookaheadIndex = std::min(currentWaypointIndex + lookaheadPoints, (int)waypoints.size() - 1);
        Vector2 target = waypoints[lookaheadIndex];
        float dx = target.x - vehicle.getX();
        float dy = target.y - vehicle.getY();
        float desired_heading = atan2(dy, dx);
        float headingError = desired_heading - vehicle.getHeading();
        float absHeadingError = abs(headingError);
        float targetSpeed;
        if (absHeadingError > 0.1f) {
            targetSpeed = 80.0f;
        } else {
            targetSpeed = 200.0f;
        }
        vehicle.setSpeed(targetSpeed);
        float steeringCommand = headingError * steeringGain;
        steeringCommand = std::clamp(steeringCommand, -0.8f, 0.8f);
        vehicle.setSteeringAngle(steeringCommand);
        float distance = sqrt(dx*dx + dy*dy);
        if (distance < 80.0f and currentWaypointIndex < waypoints.size() - 1) {
            currentWaypointIndex++;
        }
}