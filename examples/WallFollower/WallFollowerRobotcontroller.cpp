//
// Created by Jeroen on 23-4-2025.
//

#include "WallFollowerRobotcontroller.h"

#include <chrono>

#include "../../com/com.h"

WallFollowerRobotcontroller::~WallFollowerRobotcontroller() {
    delete this->motorController;
    delete this->positionTracker;
}

void WallFollowerRobotcontroller::Setup(Microsim::Robot robot, void *data) {
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

void WallFollowerRobotcontroller::Loop(float dtf) {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed = now - then;
    const long long dt = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    // const float dtf = static_cast<float>(dt) / 1000.0;
    then = now;

    positionTracker->Process(&robotPosition);
    motorController->UpdateMovement(dtf, robotPosition);

    v2f gridPos, targetGridPos;
    v2i targetPos, targetDir;

    switch (state) {
        case Start:
            gridPos = robotPosition.position / 180.0f;
            targetGridPos = gridPos.round();
            targetPos = (targetGridPos * 180).roundToV2i();
            targetDir = targetPos - robotPosition.position;
            motorController->RotateDegrees(static_cast<int>(std::round(std::atan2(targetDir.y, targetDir.x) * DEG2RAD)));
            Debug::Log("StartRotate");
            state = StartRotate;
            break;

        case StartRotate:
            if (motorController->GetMoveState() == IMotorController::Idle) {
                targetPos = (targetGridPos * 180).roundToV2i();
                targetDir = targetPos - robotPosition.position;
                motorController->MoveDistance(targetDir.length());
                Debug::Log("StartMoving");
                state = StartMoving;
            }
            break;

        case StartMoving:
            if (motorController->GetMoveState() == IMotorController::Idle) {
                motorController->RotateToAngle(static_cast<int>(round(robotPosition.angle / 90)) * 90);
                Debug::Log("Moving");
                state = Moving;
            }
            break;

        case Rotating:
            if (motorController->GetMoveState() == IMotorController::Idle) {
                gridPos = (robotPosition.position / 180.0f).round();
                targetPos = ((gridPos + v2f::fromAngle(robotPosition.angle)) * 180).roundToV2i();
                targetDir = targetPos - robotPosition.position;
                motorController->MoveDistance(targetDir.length());
                Debug::Log("Moving");
                state = Moving;
            }
            break;

        case Moving:
            if (motorController->GetMoveState() == IMotorController::Idle) {
                Debug::Log("Idle");
                state = Idle;
            }
            break;

        case Idle:
            int fwd = fwdSensor.ReadValue();
            int lhs = leftSensor.ReadValue();
            int rhs = rightSensor.ReadValue();

            gridPos = (robotPosition.position / 180.0f).round();

            if (fwd == 0 || fwd > 60) {
                targetPos = (gridPos + v2f::fromAngle(robotPosition.angle).explode().normalize()).roundToV2i() * 180;
                Debug::Log("Forward");
            } else {
                int r = static_cast<int>(round(static_cast<double>(rand()) / RAND_MAX));
                // probably possible in 1 calculation w/0 any statements
                if (r == 0) {
                    if (lhs == 0 || lhs < 60) {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle - 90).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("-90");
                    } else if (rhs == 0 || rhs < 60) {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle + 90).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("+90");
                    } else {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle + 180).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("+180");
                    }
                } else if (r == 1) {
                    if (rhs == 0 || rhs < 60) {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle + 90).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("+90");
                    } else if (lhs == 0 || lhs < 60) {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle - 90).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("-90");
                    } else {
                        targetPos = (gridPos + v2f::fromAngle(robotPosition.angle + 180).explode().normalize()).roundToV2i() * 180;
                        Debug::Log("+180");
                    }
                }
            }

            targetDir = targetPos - robotPosition.position;
            Debug::DrawRay2D(robotPosition.position.toV2f() / 1000.0, targetDir.toV2f() / 1000.0, 2, Color::green());
            Debug::DrawRay3D(targetPos.toV2f().toFlatV3f() / 1000.0, v3f(0,1,0), 2, Color::red());
            motorController->RotateDegrees(static_cast<int>(std::round(std::atan2(targetDir.y, targetDir.x) * RAD2DEG)));
            Debug::LogV2i("Target Dir:", targetDir);
            Debug::Log("Rotating");
            state = Rotating;
            break;
    }
}

