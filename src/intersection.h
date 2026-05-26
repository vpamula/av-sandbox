#pragma once
#include "raylib.h"

class Intersection {
    public:
        Intersection(Rectangle bounds);
        Rectangle getBounds() const;
        void Draw();

    private:
        Rectangle bounds;
};