# AV Sandbox

A small C++17 driving sandbox built with Raylib. The current prototype models straight two-way roads, legal road connections, point-to-point routing, and a single vehicle that follows the route.

## Build and run

Raylib and CMake are required.

```sh
cmake -S . -B build
cmake --build build
./build/av-sandbox
```

## Controls

- Click a road to choose the route start, then click a destination.
- Press Space to pause or resume.
- Press Tab to switch between automatic and manual driving.
- In manual mode, use W/S to accelerate or brake and A/D to steer.
- Drag with the right mouse button to pan; use the wheel or +/- to zoom.
- Press R to clear the route.

## Code map

- `Road` owns its two opposing `Lane` objects and draws the road surface.
- `World` owns roads and intersections, then builds the route graph.
- `Intersection` lists legal lane-to-lane movements.
- `RoadGraph` discretizes lanes into directed edges and uses Dijkstra's algorithm to find a route.
- `LaneFollower` steers the vehicle through route waypoints.
- `Vehicle` applies steering and speed to a simple bicycle-style motion model.
- `main.cpp` creates the sample world and handles input, camera, and drawing.

Curved roads, traffic signals, and other vehicles are not implemented yet.
