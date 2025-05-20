#ifndef ALGORITHMS_H
#define ALGORITHMS_H
#include "../microsim/microsim.h"

class IRobotController {
public:
	virtual ~IRobotController() = default;
	virtual void Setup(Microsim::Robot robot, void* data) = 0;
	virtual void Loop(float dtf) = 0;
};

class IMotorController {
public:
	enum MoveState : uint32_t {
            Idle,
            Turning,
            Moving
	};

	virtual ~IMotorController() = default;
	virtual void Setup(Microsim::Robot robot, void* data) = 0;
    virtual void UpdateMovement(float dt, RobotPosition position) = 0;
	virtual MoveState GetMoveState() = 0;
    virtual float GetDistanceCovered() = 0;
    virtual float GetTargetDistance() = 0;
    virtual void SetGyroNull() = 0;
	virtual void SetRpm(int rpm) = 0;
	virtual void MoveDistance(float distance) = 0;
	virtual void RotateToAngle(int wantedAngle) = 0;
	virtual void RotateDegrees(int degrees) = 0;
};

class IPositionTracker {
public:
	virtual ~IPositionTracker() = default;
	virtual void Setup(Microsim::Robot robot, void* data) = 0;
	virtual void Process(RobotPosition* robotPosition) = 0;
};

class  IObjectDetectorAlgorithm {
public:
	virtual ~IObjectDetectorAlgorithm() = default;
	virtual void Setup(Microsim::Robot robot, void* data) = 0;
	virtual void Process(Map map) = 0;
};

class IPathfinder {
public:
	virtual ~IPathfinder() = default;
	virtual void Setup(Microsim::Robot robot, void* data) = 0;
	virtual int Pathfind(Map map, RobotPosition position, v2f target, v2i* path) = 0;
};

class SimulatorMotorController : IMotorController, Microsim::Object {
public:
	SimulatorMotorController(const char* algorithm);
	~SimulatorMotorController() override;
	void Setup(Microsim::Robot robot, void* data) override;
	void UpdateMovement(float dt, RobotPosition position) override;
	MoveState GetMoveState() override;
	float GetDistanceCovered() override;
	float GetTargetDistance() override;
	void SetGyroNull() override;
	void SetRpm(int rpm) override;
	void MoveDistance(float distance) override;
	void RotateToAngle(int wantedAngle) override;
	void RotateDegrees(int degrees) override;
};

class SimulatorPositionTracker : IPositionTracker, Microsim::Object {
public:
	SimulatorPositionTracker(const char* algorithm);
	~SimulatorPositionTracker() override;
	void Setup(Microsim::Robot robot, void* data) override;
	void Process(RobotPosition* robotPosition) override;
};
#endif //ALGORITHMS_H
