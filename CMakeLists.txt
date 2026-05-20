cmake_minimum_required(VERSION 3.10)

project(av-sandbox)

set(CMAKE_CXX_STANDARD 17)

include_directories(/opt/homebrew/include)

link_directories(/opt/homebrew/lib)

add_executable(av-sandbox

    src/main.cpp

    src/vehicle.cpp

    src/road.cpp

    src/path.cpp

    src/lanefollower.cpp

    src/trajectorygenerator.cpp

    src/lane.cpp
    

)

target_link_libraries(av-sandbox raylib)