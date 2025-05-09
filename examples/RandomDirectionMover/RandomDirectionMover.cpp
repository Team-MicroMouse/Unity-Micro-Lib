//
// Created by gamin on 09/05/2025.
//

#include "RandomDirectionMover.h"
#include <chrono>

#include "../../com/com.h"

RandomDirectionMover::~RandomDirectionMover() {
    delete this->motorController;
    delete this->positionTracker;
}

void RandomDirectionMover::Setup(Microsim::Robot robot, void *data)
{
    this->robot = robot;
    fwdSensor = robot.FindComponent(Guid(4764948050219179759u, 13563840741769542562u)).ToSensor_i32();
    leftSensor = robot.FindComponent(Guid(5421639628147193954u, 6046799433377730472u)).ToSensor_i32();
    rightSensor = robot.FindComponent(Guid(5124586844430470358u, 6050595459198776716u)).ToSensor_i32();

    motorController = reinterpret_cast<IMotorController*>(new SimulatorMotorController("DefaultMotorController"));
    positionTracker = reinterpret_cast<IPositionTracker*>(new SimulatorPositionTracker("DefaultPositionTracker"));
    then = std::chrono::high_resolution_clock::now();
}

void RandomDirectionMover::Loop(float dtf)
{
    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed = now - then;
    then = now;

    positionTracker->Process(&robotPosition);
    motorController->UpdateMovement(dtf, robotPosition);
}