#ifndef ALGORITHMS_H
#define ALGORITHMS_H
#include "../microsim/microsim.h"

class IAlgorithm {
public:
};

class  IObjectDetectorAlgorithm {
public:
	virtual ~IObjectDetectorAlgorithm() = default;
	virtual void Setup(Microsim::Robot robot, void*) = 0;
	virtual void Process(int* map, v2i mapSize) = 0;
};

class TestObjectDetector : IObjectDetectorAlgorithm {
public:
	void Setup(Microsim::Robot robot, void*) override;
	void Process(int* map, v2i mapSize) override;

private:
	Microsim::Sensor_i32 fwdSensor = {};
};

#endif //ALGORITHMS_H
