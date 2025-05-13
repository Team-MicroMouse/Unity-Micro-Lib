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
    motorController->SetRpm(80);
    state = Start;


}

void RandomDirectionMover::Loop(float dtf)
{
    auto normalizedAngle = [](int angle) {
        return (angle % 360 + 360) % 360; // always between 0–359
    };

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
        state = Moving;
        UnityEngine::Log("Starting");
        break;
    case Moving:
        if ((fwd != 0 && fwd < 55) || (lhs != 0 && lhs < 30) || (rhs != 0 && rhs < 30))
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
            targetPos = ((gridPos + v2f::fromAngle(robotPosition.angle).normalize()) * 180).roundToV2i();
            targetDir = targetPos - robotPosition.position;
            motorController->MoveDistance(targetDir.length());
            UnityEngine::Log("Moving");
            state = Moving;
        }
        break;
    case Collided:
        if (motorController->GetMoveState() == IMotorController::Idle)
        {
            gridPos = (robotPosition.position / 180.0f).round();
            int roundedAngle = static_cast<int>(std::round(robotPosition.angle / 90.0) * 90) % 360;
            if (fwd == 0 || fwd > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(roundedAngle).normalize()).roundToV2i() * 180;
                UnityEngine::Log("+0");
            } else if (lhs == 0 || lhs > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(normalizedAngle(roundedAngle + 270) % 360).normalize()).roundToV2i() * 180;
                UnityEngine::Log("-90");
            } else if (rhs == 0 || rhs > 100)
            {
                targetPos = (gridPos + v2f::fromAngle(normalizedAngle(roundedAngle - 270) % 360).normalize()).roundToV2i() * 180;
                UnityEngine::Log("+90");
            } else
            {
                targetPos = (gridPos + v2f::fromAngle(roundedAngle + 180).normalize()).roundToV2i() * 180;
                UnityEngine::Log("+180");
            }

            targetDir = targetPos - robotPosition.position;
            float angle = std::atan2(targetDir.y, targetDir.x) * RAD2DEG;
            motorController->RotateToAngle(static_cast<int>(std::round(angle + 90)) % 360);
            state = Rotating;
            UnityEngine::LogV2f("gridPos", gridPos);
            UnityEngine::Logf("atan2 angle", angle);
            UnityEngine::Logi("rotateTo", roundedAngle);
            UnityEngine::Log("Rotating");
        }
        break;
    }


}