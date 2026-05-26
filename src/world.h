#pragma once

#include "road.h"
#include "intersection.h"
#include <vector>

class World {

    public:
        void addRoad(Road road);
        void computeIntersections();
        void Draw();
        std::vector<Road>& getRoads();
        std::vector<Intersection>& getIntersections();

    private:
        std::vector<Road> roads;
        std::vector<Intersection> intersections;
};