cmake_minimum_required(VERSION 3.30)
project(Unity_Link_Test)

set(CMAKE_CXX_STANDARD 20)

add_library(Unity_Link_Test SHARED library.cpp
        #        target_include_directories(Unity_Link_Test PRIVATE
#        ${CMAKE_SOURCE_DIR}/types
#        ${CMAKE_SOURCE_DIR}/examples/ObjectDetection

        com/com.cpp
        com/com.h
        microsim/microsim.cpp
        algorithms/algorithms.cpp
        types/types.cpp
        types/types.h
        examples/ObjectDetection/Objectdetection.cpp
        examples/ObjectDetection/Objectdetection.h
        examples/WallFollower/WallFollowerRobotcontroller.cpp
        examples/WallFollower/WallFollowerRobotcontroller.h
        algorithms/SimulatorMotorController.cpp
        algorithms/SimulatorPositionTracker.cpp
        examples/Floodfill/Floodfill.cpp
        examples/Floodfill/Floodfill.h
        examples/FloodfillStack/FloodfillStack.cpp
        examples/FloodfillStack/FloodfillStack.h
        examples/Astar/Astar.cpp
        examples/Astar/Astar.h
        examples/MMarc/MMarc.cpp
        examples/MMarc/MMarc.h
        examples/Tawd/Tawd.cpp
        examples/Tawd/Tawd.h
        examples/Dijkstra/Dijkstra.cpp
        examples/Dijkstra/Dijkstra.h
)
