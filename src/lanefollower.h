#pragma once
#include "vehicle.h"
#include "raylib.h"
#include <vector>

class LaneFollower {
    public:
        LaneFollower();
        void Update(Vehicle& vehicle, const std::vector<Vector2>& waypoints, float dt);
        void reacquireWaypoint(Vehicle& vehicle,const std::vector<Vector2>& waypoints);

    private:
        int currentWaypointIndex;
        float steeringGain;
        int lookaheadPoints;

};