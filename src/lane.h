#pragma once
#include "raylib.h"
#include <vector>

class Road;
class Lane {
    public:
        Lane(float center_x, float center_y, float heading, float width, float length);
        std::vector<Vector2> generateWaypoints();
        Vector2 getDirection();
        int getId() const {
            return id;
        }
        float getCenterX() {
            return center_x;
        }
        float getCenterY() {
            return center_y;
        }

        Vector2 getStartPoint() {
            Vector2 dir = getDirection();
            return {center_x - dir.x * length/2, center_y - dir.y * length/2};
        }

        Vector2 getEndPoint() {
            Vector2 dir = getDirection();
            return {center_x + dir.x * length/2, center_y + dir.y * length/2};
        }

        std::vector<Vector2> generateWaypointsBetween(Vector2 startPoint, Vector2 endPoint);

        void setParentRoad(Road* road) {
            parentRoad = road;
        }

        Road* getParentRoad() {
            return parentRoad;
        }

    private:
        int id;
        static int nextId;
        float center_x;
        float center_y;
        float heading;
        float width;
        float length;
        Road* parentRoad = nullptr;
};