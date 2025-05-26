#include "com.h"

#include <cstdlib>
#include <iostream>

#include "../algorithms/algorithms.h"
#include "../examples/Floodfill/Floodfill.h"
#include "../examples/FloodfillStack/FloodfillStack.h"
#include "../examples/SimpleObjectDetector/SimpleObjectDetector.h"
#include "../examples/WallFollower/WallFollowerRobotcontroller.h"
#include "../microsim/microsim.h"

class FloodfillStack;
Plugin::CreateNativeObject* nativeObjectConstructorList;
int nativeObjectListPtr;

void* Plugin::Plugin_CreateObject(const uint32_t idx) {
	return nativeObjectConstructorList[idx]();
}

void Plugin::Plugin_DeleteObject(void* handle) {
	free(handle);
}

void Microsim::ObjectDetector_Setup(IObjectDetectorAlgorithm* algorithm, uint32_t robotHandle, void* data) {
	if (algorithm == nullptr) {
		UnityEngine::Log("Nullptr in setup");
	} else {
		const Robot robot = { robotHandle };
		algorithm->Setup(robot, data);
	}
}

void Microsim::ObjectDetector_Process(IObjectDetectorAlgorithm* algorithm, int *map, v2i mapSize) {
	if (algorithm == nullptr) {
		UnityEngine::Log("Nullptr in ObjectDetector_Process");
	} else {
		algorithm->Process(map, mapSize);
	}
}

void Microsim::Pathfinder_Setup(IPathfinder* algorithm, uint32_t robotHandle, void* data) {
	if (algorithm == nullptr) {
		UnityEngine::Log("Nullptr in Pathfinder_setup");
	} else {
		algorithm->Setup(Robot { robotHandle }, data);
	}
}

int Microsim::Pathfinder_Pathfind(IPathfinder* ptr, Map map, RobotPosition position, v2f target, v2i* path) {
	if (ptr == nullptr) {
		UnityEngine::Log("Nullptr in Pathfinder_Pathfind");
		return -1;
	} else {
		return ptr->Pathfind(map, position, target, path);
	}
}

void Microsim::RobotController_Setup(IRobotController* robotController, uint32_t robotHandle, void* data) {
	if (robotController == nullptr) {
		UnityEngine::Log("Nullptr in RobotController_Setup");
	} else {
		const Robot robot = { robotHandle };
		robotController->Setup(robot, data);
	}
}

void Microsim::RobotController_Loop(IRobotController* robotController, float dtf) {
	if (robotController == nullptr) {
		UnityEngine::Log("Nullptr in RobotController_Process");
	} else {
		robotController->Loop(dtf);
	}
}

template<typename T> void registerObject(const Plugin::NativeObjectType type, const char* name) {
	const Plugin::CreateNativeObject fn = [&] { return new T(); };
	const uint32_t idx = nativeObjectListPtr;
	nativeObjectConstructorList[nativeObjectListPtr] = fn;
	Plugin::RegisterType(Plugin::NativeObjectFactory {type, idx, typeid(T).name(), name });
	nativeObjectListPtr++;
}

void Init(uint8_t* (*getFunction)(const char* name)) {
	GetFunction = getFunction;
	nativeObjectConstructorList = static_cast<Plugin::CreateNativeObject *>(calloc(1024, sizeof(Plugin::CreateNativeObject)));

	/* Registering plugin data */

	Plugin::RegisterData = *reinterpret_cast<void (**)(const char*, const char*)>(GetFunction("Plugin::RegisterData"));
	Plugin::RegisterData("TestLib", "0.0.1");

	/* Registering functions */

	UnityEngine::Log = *reinterpret_cast<void (**)(const char *)>(GetFunction("UnityEngine::Log"));
	UnityEngine::Log("Gathering Simulator Functions");

	UnityEngine::Logi = *reinterpret_cast<void (**)(const char *, int)>(GetFunction("UnityEngine::Logi"));
	UnityEngine::Logf = *reinterpret_cast<void (**)(const char *, float)>(GetFunction("UnityEngine::Logf"));
	UnityEngine::LogV2f = *reinterpret_cast<void (**)(const char *, v2f)>(GetFunction("UnityEngine::LogV2f"));
	UnityEngine::LogV2i = *reinterpret_cast<void (**)(const char *, v2i)>(GetFunction("UnityEngine::LogV2i"));

	UnityEngine::Debug::DrawLine2D = *reinterpret_cast<void (**)(v2f, v2f, float, Color)>(GetFunction("UnityEngine::Debug::DrawRay2D"));
	UnityEngine::Debug::DrawLine3D = *reinterpret_cast<void (**)(v3f, v3f, float, Color)>(GetFunction("UnityEngine::Debug::DrawRay3D"));
	UnityEngine::Debug::DrawRay2D = *reinterpret_cast<void (**)(v2f, v2f, float, Color)>(GetFunction("UnityEngine::Debug::DrawLine2D"));
	UnityEngine::Debug::DrawRay3D = *reinterpret_cast<void (**)(v3f, v3f, float, Color)>(GetFunction("UnityEngine::Debug::DrawLine3D"));

	Plugin::RegisterType = *reinterpret_cast<void (**)(Plugin::NativeObjectFactory)>(GetFunction("Plugin::RegisterType"));

	Microsim::Sensor_i32_ReadValue = *reinterpret_cast<int32_t (**)(uint32_t)>(GetFunction("Microsim::Sensor_i32_ReadValue"));
	Microsim::Sensor_f32_ReadValue = *reinterpret_cast<float (**)(uint32_t)>(GetFunction("Microsim::Sensor_f32_ReadValue"));
	Microsim::Sensor_v3i_ReadValue = *reinterpret_cast<v3i (**)(uint32_t)>(GetFunction("Microsim::Sensor_v3i_ReadValue"));
	Microsim::Motor_CurrentThrottle = *reinterpret_cast<int8_t (**)(uint32_t)>(GetFunction("Microsim::Motor_CurrentThrottle"));
	Microsim::Motor_SetThrottle = *reinterpret_cast<void (**)(uint32_t, int8_t)>(GetFunction("Microsim::Motor_SetThrottle"));
	Microsim::Robot_FindComponent = *reinterpret_cast<uint32_t (**)(uint32_t, Guid)>(GetFunction("Microsim::Robot_FindComponent"));

	Microsim::SimMotorController_Setup = *reinterpret_cast<void (**)(uint32_t handle, uint32_t robotHandle, void* data)>(GetFunction("Microsim::SimMotorController_Setup"));
	Microsim::SimMotorController_UpdateMovement = *reinterpret_cast<void (**)(uint32_t handle, float dt, RobotPosition position)>(GetFunction("Microsim::SimMotorController_UpdateMovement"));
	Microsim::SimMotorController_GetCurrentState = *reinterpret_cast<uint32_t (**)(uint32_t handle)>(GetFunction("Microsim::SimMotorController_GetCurrentState"));
	Microsim::SimMotorController_GetDistanceCovered = *reinterpret_cast<float (**)(uint32_t handle)>(GetFunction("Microsim::SimMotorController_GetDistanceCovered"));
	Microsim::SimMotorController_GetTargetDistance = *reinterpret_cast<float (**)(uint32_t handle)>(GetFunction("Microsim::SimMotorController_GetTargetDistance"));
	Microsim::SimMotorController_SetGyroNull = *reinterpret_cast<void (**)(uint32_t handle)>(GetFunction("Microsim::SimMotorController_SetGyroNull"));
	Microsim::SimMotorController_SetRpm = *reinterpret_cast<void (**)(uint32_t handle, int rpm)>(GetFunction("Microsim::SimMotorController_SetRpm"));
	Microsim::SimMotorController_MoveDistance = *reinterpret_cast<void (**)(uint32_t handle, float distance)>(GetFunction("Microsim::SimMotorController_MoveDistance"));
	Microsim::SimMotorController_RotateToAngle = *reinterpret_cast<void (**)(uint32_t handle, int wantedAngle)>(GetFunction("Microsim::SimMotorController_RotateToAngle"));
	Microsim::SimMotorController_RotateDegrees = *reinterpret_cast<void (**)(uint32_t handle, int degrees)>(GetFunction("Microsim::SimMotorController_RotateDegrees"));

	Microsim::SimPositionTracker_Setup = *reinterpret_cast<void (**)(uint32_t handle, uint32_t robotHandle, void* data)>(GetFunction("Microsim::SimPositionTracker_Setup"));
	Microsim::SimPositionTracker_Process = *reinterpret_cast<void (**)(uint32_t handle, RobotPosition* robotPosition)>(GetFunction("Microsim::SimPositionTracker_Process"));

	Microsim::CreateSimulatorAlgorithm = *reinterpret_cast<uint32_t (**)(const char*)>(GetFunction("Microsim::CreateSimulatorAlgorithm"));
	Microsim::FreeSimulatorAlgorithm = *reinterpret_cast<void (**)(uint32_t handle)>(GetFunction("Microsim::FreeSimulatorAlgorithm"));

	/* Registering objects */

	UnityEngine::Log("Registering Native Objects");

	registerObject<SimpleObjectDetector>(Plugin::ObjectDetector, "Simple Object Detector");
	registerObject<WallFollowerRobotcontroller>(Plugin::RobotController, "Simple Robot Controller");
	registerObject<Floodfill>(Plugin::Pathfinder, "Flood Fill");
	registerObject<FloodfillStack>(Plugin::Pathfinder, "Flood Fill Stack");

	/* Finishing up */

	UnityEngine::Log("Successfully loaded C++ Test Plugin");
}

void Destroy() {
	free(nativeObjectConstructorList);
}