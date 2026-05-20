#pragma once

#include "raylib.h"
#include "lane.h"
#include <vector>

class Road {
    public: 
        Road(float center_x, float center_y, float heading, float width, float length);
        void Draw();
        Lane& getRightLane();
        Lane& getLeftLane();
        float getWidth() {
            return width;
        }
        std::vector<Vector2> generateLaneWaypoints(bool rightLane);

    private:
        float center_x;
        float center_y;
        float heading;
        float width;
        float length;
        Lane rightLane;
        Lane leftLane;

};