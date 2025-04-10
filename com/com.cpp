#include "com.h"

#include <cstdlib>
#include <string>

#include "../algorithms/algorithms.h"
#include "../microsim/microsim.h"

void* Plugin::Plugin_CreateObject(const uint32_t idx) {
	return nullptr;
}

void Plugin::Plugin_DeleteObject(void* handle) {
	free(handle);
}

void Microsim::Algorithm_Setup(IAlgorithm* algorithm, void *data) {
	if (algorithm == nullptr) {
		UnityEngine::Log("Nullptr in setup");
	} else {
		algorithm->Setup(data);
	}
}

void Microsim::ObjectDetector_Process(IObjectDetectorAlgorithm* ptr, int *map, int mapSize) {

}

template<typename T> void registerObject(const Plugin::NativeObjectType type, const char* name) {
	const Plugin::CreateNativeObject fn = [&] { return new T(); };
	// const uint32_t idx = COL->size();
	Plugin::RegisterType(Plugin::NativeObjectFactory {type, 0, typeid(T).name(), name });
	// COL->push_back(fn);
}

void Init(uint8_t* (*getFunction)(const char* name)) {
	GetFunction = getFunction;

	UnityEngine::Log = *reinterpret_cast<void (**)(const char *)>(GetFunction("UnityEngine::Log"));
	UnityEngine::Log("Gathering Simulator Functions");

	UnityEngine::Logi = *reinterpret_cast<void (**)(const char *, int)>(GetFunction("UnityEngine::Logi"));

	Plugin::RegisterType = *reinterpret_cast<void (**)(Plugin::NativeObjectFactory)>(GetFunction("Plugin::RegisterType"));

	Microsim::Sensor_ReadValue = *reinterpret_cast<int32_t (**)(uint32_t)>(GetFunction("Microsim::Sensor_ReadValue"));
	Microsim::Motor_CurrentThrottle = *reinterpret_cast<int8_t (**)(uint32_t)>(GetFunction("Microsim::Sensor_ReadValue"));
	Microsim::Motor_SetThrottle = *reinterpret_cast<void (**)(uint32_t, int8_t)>(GetFunction("Microsim::Sensor_ReadValue"));

	UnityEngine::Log("Registering Native Objects");

	registerObject<TestObjectDetector>(Plugin::NativeObjectType::ObjectDetector, "Test Object Detector");

	Sensor sensor;
	sensor.handle = 0;
	UnityEngine::Logi("Reading sensor value", sensor.ReadValue());
	UnityEngine::Log("Successfully loaded C++ Test Plugin");
}
