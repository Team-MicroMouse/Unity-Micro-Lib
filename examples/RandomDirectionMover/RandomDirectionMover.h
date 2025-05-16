//
// Created by gamin on 09/05/2025.
//

#ifndef RandomDirectionMover_H
#define RandomDirectionMover_H

#include <chrono>
#include "../../algorithms/algorithms.h"
#include "../../microsim/microsim.h"



class RandomDirectionMover: IRobotController {
    enum State {
        Start,
        Moving,
        Rotating,
        Collided,
        Idle,
    };

    std::chrono::steady_clock::time_point then;

    Microsim::Robot robot;
    Microsim::Sensor_i32 fwdSensor, leftSensor, rightSensor;
    RobotPosition robotPosition;

    IPositionTracker* positionTracker;
    IMotorController* motorController;

    State state;

public:
    ~RandomDirectionMover() override;
    void Setup(Microsim::Robot robot, void* data) override;
    void Loop(float dtf) override;
};

#endif
