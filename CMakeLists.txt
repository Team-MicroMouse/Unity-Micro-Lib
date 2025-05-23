cmake_minimum_required(VERSION 3.30)
project(Unity_Link_Test)

set(CMAKE_CXX_STANDARD 20)

add_library(Unity_Link_Test SHARED library.cpp
        examples/ObjectDetection/Direction.h)
        target_include_directories(Unity_Link_Test PRIVATE
        ${CMAKE_SOURCE_DIR}/types
        ${CMAKE_SOURCE_DIR}/examples/ObjectDetection

        com/com.cpp
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
        examples/ObjectDetection/Objectdetection.cpp
        examples/ObjectDetection/Objectdetection.h
)
