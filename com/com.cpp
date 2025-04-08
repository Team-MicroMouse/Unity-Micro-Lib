#include "com.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include "../algorithms/algorithms.h"

std::list<Plugin::CreateNativeObject> createObjects = {};

void* Plugin::Plugin_CreateObject(const uint32_t idx) {
	CreateNativeObject fn = *std::next(createObjects.begin(), idx);
	return fn();
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

void Microsim::ObjectDetector_Process(void* ptr, int *map, int mapSize) {

}

template<typename T> void registerObject(const Plugin::NativeObjectType type, const char* name) {
	const Plugin::CreateNativeObject fn = [] () { return new T(); };
	Plugin::RegisterType(Plugin::NativeObjectFactory {type, static_cast<uint32_t>(createObjects.size()), typeid(T).name(), name });
	createObjects.push_back(fn);
}

void Init(uint8_t* (*getFunction)(const char* name)) {
	GetFunction = getFunction;

	UnityEngine::Log = *reinterpret_cast<void (**)(const char *)>(GetFunction("UnityEngine::Log"));
	UnityEngine::Log("Gathering Simulator Functions");

	Plugin::RegisterType = *reinterpret_cast<void (**)(Plugin::NativeObjectFactory)>(GetFunction("Plugin::RegisterType"));

	UnityEngine::Log("Registering Native Objects");

	registerObject<TestObjectDetector>(Plugin::NativeObjectType::ObjectDetector, "Test Object Detector");

	UnityEngine::Log("Successfully loaded C++ Test Plugin");
}
