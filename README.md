# AV Sandbox

A small C++17 driving sandbox built with Raylib. The sample world is a two-way downtown grid with five blocks in each direction, several curved streets, point-to-point routing, and one vehicle that follows the route.

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

- `Road` stores a sampled centerline, creates two offset lane paths, and draws straight or curved road surfaces.
- `World::makeDowntownGrid` creates the sample city; `World` finds centerline crossings and builds the route graph.
- `Intersection` lists legal lane-to-lane movements and renders stop-sign or traffic-light controls.
- `RoadGraph` keeps each lane as directed edges, adds explicit spline connections at intersections, and uses Dijkstra's algorithm to find a route.
- `LaneFollower` steers through route waypoints and obeys stop signs and traffic lights.
- `Vehicle` applies steering and speed to a simple bicycle-style motion model.
- `main.cpp` creates the sample world and handles input, camera, and drawing.

The current world uses sampled sinusoidal curves to create gently bending streets. Six intersections get controls: three stop-sign junctions and three traffic-light junctions. Their placement is randomized, with no two controls sharing a grid row or column. NPC cars follow generated lane routes, obey stop signs and traffic lights, and slow for a vehicle ahead. Full vehicle collision physics are not implemented yet.
