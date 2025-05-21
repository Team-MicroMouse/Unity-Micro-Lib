
#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include "../../algorithms/algorithms.h"

class Objectdetection : public IObjectDetectorAlgorithm {
public:
    void Setup(Microsim::Robot robot, void* data)override;
    void Process(Map map, RobotPosition position)override;

private:
    Microsim::Sensor_i32 fwdSensor;
    Microsim::Sensor_i32 rightSensor;
    Microsim::Sensor_i32 leftSensor;

#ifdef DEBUG_MODE
    void LogSensors(int front, int left, int right);
#endif
};
#endif //OBJECTDETECTION_H
