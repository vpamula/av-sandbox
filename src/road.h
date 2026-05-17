#pragma once

#include "raylib.h"

class Road {
    public: 
        Road(float center_x, float center_y, float heading, float width, float length);
        void Draw();

    private:
        float center_x;
        float center_y;
        float heading;
        float width;
        float length;


};