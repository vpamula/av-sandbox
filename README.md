# AV Sandbox

A small C++17 driving simulator built with Raylib. It creates a downtown-style road network with straight and curved streets, lane-level routing, traffic controls, NPC cars, cyclists, pedestrians, a simple perception model, and predictive collision-avoidance behavior.

This is a visual planning and behavior demo. Its kinematic vehicle model, simulated perception, and safety logic are intentionally simplified; they are not a production AV stack or a validated safety system.

## Build and run

Requirements: CMake and Raylib (available through CMake package discovery).

```sh
cmake -S . -B build
cmake --build build
./build/av-sandbox
```

## Controls

- Click a road to set the first route point, then keep clicking to add route points in order.
- The blue path shows the vehicle's approach to the first selected point; the yellow path connects selected points.
- Press `Space` to pause or resume. The camera remains manually controllable while paused.
- Press `Tab` to switch between autonomous and manual driving.
- In manual mode, use `W`/`S` to accelerate or brake and `A`/`D` to steer.
- Drag with the right or middle mouse button to pan; left-button dragging also pans.
- Use the mouse wheel or `+`/`-` to zoom.
- Press `R` to clear the route.

## Current world

The sample world is a five-by-five-block grid with six roads in each direction and 36 intersections. Streets are represented by sampled centerlines; some use gentle curves. A few roads have two lanes per direction. The world also draws bike lanes, sidewalks, intersection paving, and crosswalks.

Traffic controls are placed at a subset of intersections: five traffic lights and five stop signs, with positions spread across the map and randomized on startup. Each traffic light runs its own timed cycle. Stop-sign traffic is coordinated FIFO. The simulation starts with 12 NPC cars, four cyclists, and six pedestrians (subject to available valid spawn locations).

## Routing and road users

- Each click snaps to the nearest lane waypoint. Multi-click routes are solved segment by segment from the previous segment's snapped endpoint, so adjacent segments stay connected.
- The directed lane graph uses waypoints as nodes and forward travel as edges. Curved connector edges represent legal intersection turns. Connectors can start or end at either of the nearest usable lane waypoints beside an intersection, which supports clicks just outside an intersection. U-turns are rejected; left turns require the leftmost approach lane and right turns require the rightmost. On multilane roads, lane changes connect adjacent lanes between intersections.
- Dijkstra minimizes traveled distance plus a small capped cost for junction paths that overlap bike lanes or crosswalks. That cost can break close ties without sending a vehicle on a long detour; a crosswalk remains traversable when required by the turn.
- Sidewalks and crosswalks are not vehicle-graph edges. Pedestrian NPCs follow separate sidewalk and crosswalk paths; cyclists follow bike-lane paths.
- NPC cars choose routes through the same lane graph. A separate traffic-safety layer predicts short-horizon motion and applies braking or yielding for potential conflicts with vehicles, cyclists, and pedestrians. At uncontrolled junctions, cars reserve entry according to predicted arrival time; the reservation can be reassigned before entry as arrival estimates change and is held until the owner clears the junction. The model uses simplified vehicle shapes and simulator state, so it is not a validated collision-prevention system.
- The perception component filters simulated actors by range, field of view, and road-corridor visibility, then adds position noise. The ego vehicle uses those observations for a basic following-speed limit; this is not camera or lidar perception.

## Code map

- `src/world.cpp`: sample-world construction, intersection generation, route controls, and graph setup.
- `src/road.cpp`, `src/lane.cpp`: sampled road geometry, lane centerlines, bike lanes, sidewalk paths, and drawing.
- `src/intersection.cpp`: legal lane-to-lane movements, intersection geometry, crosswalk generation, stop signs, and traffic lights.
- `src/roadgraph.cpp`: directed lane graph, lane changes, intersection connectors, route costs, click snapping, and Dijkstra search.
- `src/lanefollower.cpp`, `src/vehicle.cpp`: waypoint following, control-point stopping, and simple bicycle-style vehicle motion.
- `src/npctraffic.cpp`, `src/npcroadusers.cpp`: NPC vehicle and vulnerable-road-user routes, spawning, and movement.
- `src/perception.cpp`, `src/trafficsafety.cpp`: simulated observations and predictive conflict handling.
- `src/stopsigncoordinator.cpp`: FIFO stop-sign coordination.
- `src/main.cpp`: input, simulation update order, camera, and rendering.
