
#ifndef TAWD_H
#define TAWD_H

#include "../../algorithms/algorithms.h"
#include "../../types/types.h"

class Tawd : IObjectDetectorAlgorithm {
public:
    void Setup(Microsim::Robot robot, void* data)override;
    void Process(Map map, RobotPosition robot_position)override;

private:
    Microsim::Sensor_i32 fwdSensor = {};
    Microsim::Sensor_i32 rightSensor = {};
    Microsim::Sensor_i32 leftSensor = {};
};
#endif //TAWD_H
