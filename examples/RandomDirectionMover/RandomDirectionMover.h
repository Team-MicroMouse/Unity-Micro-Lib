//
// Created by gamin on 09/05/2025.
//

#ifndef RANDOMDIRECTIONMOVER_H
#define RANDOMDIRECTIONMOVER_H

#include <chrono>
#include "../../algorithms/algorithms.h"
#include "../../microsim/microsim.h"

enum State {
    Start,
    Moving,
    Rotating,
    Idle,
};

class RandomDirectionMover: IRobotController {
    std::chrono::steady_clock::time_point then;

    Microsim::Robot robot;
    Microsim::Sensor_i32 fwdSensor, leftSensor, rightSensor;
    RobotPosition robotPosition;

    IPositionTracker* positionTracker;
    IMotorController* motorController;

public:
    ~RandomDirectionMover() override;
    void Setup(Microsim::Robot robot, void* data) override;
    void Loop(float dtf) override;
};

#endif //RANDOMDIRECTIONMOVER_H
