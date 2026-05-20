#pragma once
#include "raylib.h"
#include <vector>

class TrajectoryGenerator {
    public:
        static std::vector<Vector2>generateConnector(Vector2 startPoint, Vector2 startDirection, Vector2 endPoint, 
            Vector2 endDirection, float handleLength, int numPoints);
};

