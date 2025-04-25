
#include "algorithms.h"
#include "../com/com.h"

SimulatorPositionTracker::SimulatorPositionTracker(const char *algorithm) {
    handle = Microsim::CreateSimulatorAlgorithm(algorithm);
}

SimulatorPositionTracker::~SimulatorPositionTracker() {
    Microsim::FreeSimulatorAlgorithm(handle);
}

void SimulatorPositionTracker::Setup(Microsim::Robot robot, void* data) {
    Microsim::SimPositionTracker_Setup(handle, robot.handle, data);
}

void SimulatorPositionTracker::Process(RobotPosition* robotPosition) {
    Microsim::SimPositionTracker_Process(handle, robotPosition);
}

