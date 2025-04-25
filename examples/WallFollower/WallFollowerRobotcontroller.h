//
// Created by Jeroen on 23-4-2025.
//

#ifndef WALLFOLLOWERROBOTCONTROLLER_H
#define WALLFOLLOWERROBOTCONTROLLER_H
#include <chrono>

#include "../../algorithms/algorithms.h"
#include "../../microsim/microsim.h"

enum State {
    Start,
    StartRotate,
    StartMoving,
    Rotating,
    Moving,
    Idle
};

class WallFollowerRobotcontroller : IRobotController {
    std::chrono::time_point<std::chrono::system_clock> then;

    Microsim::Robot robot;
    Microsim::Sensor_i32 fwdSensor, leftSensor, rightSensor;
    RobotPosition robotPosition;

    State state;

    IPositionTracker* positionTracker;
    IMotorController* motorController;

public:
    ~WallFollowerRobotcontroller() override;
    void Setup(Microsim::Robot robot, void* data) override;
    void Loop(float dtf) override;
};



#endif //WALLFOLLOWERROBOTCONTROLLER_H
