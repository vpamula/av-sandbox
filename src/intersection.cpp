#include "intersection.h"

Intersection::Intersection(Rectangle bounds) {
    this->bounds = bounds;
}

Rectangle Intersection::getBounds() const {
    return bounds;
}

void Intersection::Draw() {
    DrawRectangleRec(bounds,GRAY);
}