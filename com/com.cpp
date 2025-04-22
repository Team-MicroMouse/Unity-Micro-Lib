#include "com.h"

#include <cstdlib>
#include <iostream>

#include "../algorithms/algorithms.h"
#include "../microsim/microsim.h"

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
		Robot robot = { robotHandle };
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

	Plugin::RegisterType = *reinterpret_cast<void (**)(Plugin::NativeObjectFactory)>(GetFunction("Plugin::RegisterType"));

	Microsim::Sensor_i32_ReadValue = *reinterpret_cast<int32_t (**)(uint32_t)>(GetFunction("Microsim::Sensor_i32_ReadValue"));
	Microsim::Sensor_f32_ReadValue = *reinterpret_cast<float (**)(uint32_t)>(GetFunction("Microsim::Sensor_f32_ReadValue"));
	Microsim::Sensor_v3i_ReadValue = *reinterpret_cast<v3i (**)(uint32_t)>(GetFunction("Microsim::Sensor_v3i_ReadValue"));
	Microsim::Motor_CurrentThrottle = *reinterpret_cast<int8_t (**)(uint32_t)>(GetFunction("Microsim::Motor_CurrentThrottle"));
	Microsim::Motor_SetThrottle = *reinterpret_cast<void (**)(uint32_t, int8_t)>(GetFunction("Microsim::Motor_SetThrottle"));
	Microsim::Robot_FindComponent = *reinterpret_cast<uint32_t (**)(uint32_t, Guid)>(GetFunction("Microsim::Robot_FindComponent"));

	/* Registering objects */

	UnityEngine::Log("Registering Native Objects");

	registerObject<TestObjectDetector>(Plugin::NativeObjectType::ObjectDetector, "Test Object Detector");

	/* Finishing up */

	UnityEngine::Log("Successfully loaded C++ Test Plugin");
}

void Destroy() {
	free(nativeObjectConstructorList);
}