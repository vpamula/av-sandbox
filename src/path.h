#pragma once
#include "raylib.h"
#include <vector>

class Path {
    public:
        void addWaypoint(Vector2 point);
        std::vector<Vector2>& getWaypoints();
        void Draw();

    private:
        std::vector<Vector2> waypoints;
};