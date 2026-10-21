cmake_minimum_required(VERSION 3.16)

project(av-sandbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib CONFIG REQUIRED)

add_executable(av-sandbox
    src/main.cpp
    src/npctraffic.cpp
    src/vehicle.cpp
    src/road.cpp
    src/lanefollower.cpp
    src/lane.cpp
    src/roadgraph.cpp
    src/world.cpp
    src/intersection.cpp
)

target_link_libraries(av-sandbox PRIVATE raylib)
