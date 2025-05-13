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

    motorController->Setup(robot, nullptr);
    positionTracker->Setup(robot, nullptr);
    motorController->SetGyroNull();
    motorController->SetRpm(40);
    state = Start;
}

void RandomDirectionMover::Loop(float dtf)
{
    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed = now - then;
    then = now;

    positionTracker->Process(&robotPosition);
    motorController->UpdateMovement(dtf, robotPosition);

    v2f gridPos, targetGridPos;
    v2i targetPos, targetDir;

    int fwd = fwdSensor.ReadValue(), lhs = leftSensor.ReadValue(), rhs = rightSensor.ReadValue();

    switch (state)
    {
    case Start:
            gridPos = robotPosition.position / 180.0f;
        targetGridPos = gridPos.round();
        targetPos = (targetGridPos * 180).roundToV2i();
        targetDir = targetPos - robotPosition.position;
        state = Collided;
        UnityEngine::Log("Starting");
        break;
    case Moving:
        if ((fwd != 0 && fwd < 60) || (lhs != 0 && lhs < 30) || (rhs != 0 && rhs < 30))
        {
            UnityEngine::Log("STOP");
            motorController->Stop();
            state = Collided;
            break;
        } else if (motorController->GetMoveState() == IMotorController::Idle)
        {
            UnityEngine::Log("Idle");
            state = Collided;
            break;
        }
        break;
    case Rotating:
        if (motorController->GetMoveState() == IMotorController::Idle)
        {
            gridPos = (robotPosition.position / 180.0f).round();
            int roundedAngle = static_cast<int>(round(robotPosition.angle / 90.0) * 90);
            v2i direction;

            if (roundedAngle == 0)
            {
                direction = v2i(0, -1);
            } else if (roundedAngle == 90 || roundedAngle -270)
            {
                direction = v2i(1, 0);
            } else if (roundedAngle == 180 || roundedAngle == -180)
            {
                direction = v2i(0, 1);
            } else if (roundedAngle == -90 || roundedAngle == 270)
            {
                direction = v2i(-1, 0);
            }
            targetPos = (gridPos + direction).roundToV2i() * 180;
            targetDir = targetPos - robotPosition.position;
            UnityEngine::Log("Moving");
            state = Moving;
        }
        break;
    case Collided:
        if (motorController->GetMoveState() == IMotorController::Idle)
        {
            gridPos = (robotPosition.position / 180.0f).round();
            int roundedAngle = static_cast<int>(std::round(robotPosition.angle / 90.0) * 90);
            if (fwd == 0 || fwd > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(roundedAngle).normalize()).roundToV2i() * 180;
                UnityEngine::Log("+0");
            } else if (lhs == 0 || lhs > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(roundedAngle - 90).normalize()).roundToV2i() * 180;
                UnityEngine::Log("-90");
            } else if (rhs == 0 || rhs > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(roundedAngle + 90).normalize()).roundToV2i() * 180;
                UnityEngine::Log("+90");
            } else
            {
                targetPos = v2i(0, 1) * 180;
                UnityEngine::Log("+180");
            }

            targetDir = targetPos - robotPosition.position;
            motorController->RotateToAngle(static_cast<int>(std::round(std::atan2(targetDir.y, targetDir.x) * RAD2DEG) + 90));
            state = Rotating;
            UnityEngine::LogV2f("gridPos", gridPos);
            UnityEngine::Logi("roundedAng", roundedAngle);
            UnityEngine::LogV2i("targetPos", targetPos);
            UnityEngine::LogV2i("targetDir", targetDir);
            UnityEngine::Log("Rotating");
        }
        break;
    }
}