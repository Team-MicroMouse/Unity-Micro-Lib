#include "../microsim/microsim.h"
#include "algorithms.h"
#include "../com/com.h"


SimulatorMotorController::SimulatorMotorController(const char *algorithm) : Object() {
    handle = Microsim::CreateSimulatorAlgorithm(algorithm);
}

SimulatorMotorController::~SimulatorMotorController() {
    Microsim::FreeSimulatorAlgorithm(handle);
}

void SimulatorMotorController::Setup(Microsim::Robot robot, void* data) {
    Microsim::SimMotorController_Setup(handle, robot.handle, data);
}

void SimulatorMotorController::UpdateMovement(float dt, RobotPosition position) {
    Microsim::SimMotorController_UpdateMovement(handle, dt, position);
}

IMotorController::MoveState SimulatorMotorController::GetMoveState() {
    return static_cast<MoveState>(Microsim::SimMotorController_GetCurrentState(handle));
}

float SimulatorMotorController::GetDistanceCovered() {
    return Microsim::SimMotorController_GetDistanceCovered(handle);
}

float SimulatorMotorController::GetTargetDistance() {
    return Microsim::SimMotorController_GetTargetDistance(handle);
}

void SimulatorMotorController::SetGyroNull() {
    Microsim::SimMotorController_SetGyroNull(handle);
}

void SimulatorMotorController::SetRpm(int rpm) {
    Microsim::SimMotorController_SetRpm(handle, rpm);
}

void SimulatorMotorController::MoveDistance(float distance) {
    Microsim::SimMotorController_MoveDistance(handle, distance);
}

void SimulatorMotorController::RotateToAngle(int wantedAngle) {
    UnityEngine::Logi("Wanted Angle:", wantedAngle);
    Microsim::SimMotorController_RotateToAngle(handle, wantedAngle);
}

void SimulatorMotorController::RotateDegrees(int degrees) {
    Microsim::SimMotorController_RotateDegrees(handle, degrees);
}