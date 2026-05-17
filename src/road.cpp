#include "road.h"
#include <cmath>

Road::Road(float center_x, float center_y, float heading, float width, float length) {
    this->center_x = center_x;
    this->center_y = center_y;
    this->heading = heading;
    this->width = width;
    this->length = length;
}

void Road::Draw() {
    Rectangle rect = {center_x, center_y, length, width};
    Vector2 origin = {length / 2, width / 2};
    DrawRectanglePro(rect, origin, heading * RAD2DEG, DARKGRAY);

    float dashLength = 80.0f;
    float dashGap = 60.0f;
    float dashWidth = 8.0f;

    for (float i = center_x - length / 2; i < center_x + length / 2;i += dashLength + dashGap) {
        Rectangle dash = {i, center_y, dashLength, dashWidth};
        Vector2 dashOrigin = {dashLength / 2, dashWidth / 2};
        DrawRectanglePro(dash, dashOrigin, heading * RAD2DEG, YELLOW);
    }

}