#include "trajectorygenerator.h"

std::vector<Vector2> TrajectoryGenerator::generateConnector(Vector2 startPoint, Vector2 startDirection, Vector2 endPoint, 
    Vector2 endDirection, float handleLength, int numPoints) {
    std::vector<Vector2> points;
    Vector2 p0 = startPoint;
    Vector2 p1 = {startPoint.x + startDirection.x * handleLength, startPoint.y + startDirection.y * handleLength};
    Vector2 p2 = {endPoint.x - endDirection.x * handleLength, endPoint.y - endDirection.y * handleLength};
    Vector2 p3 = endPoint;
    for (int i = 0; i <= numPoints; i++) {
        float t = (float)i / numPoints;
        float u = 1.0f - t;
        float x = u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x;
        float y = u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y;
        points.push_back((Vector2){x, y});
    }
    return points;
}