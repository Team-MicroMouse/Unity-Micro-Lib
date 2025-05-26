cmake_minimum_required(VERSION 3.30)
project(Unity_Link_Test)

set(CMAKE_CXX_STANDARD 20)

add_library(Unity_Link_Test SHARED library.cpp
        com/com.cpp
        microsim/microsim.cpp
        algorithms/algorithms.cpp
        types/types.cpp
        types/types.h
        examples/SimpleObjectDetector/SimpleObjectDetector.cpp
        examples/SimpleObjectDetector/SimpleObjectDetector.h
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
)
