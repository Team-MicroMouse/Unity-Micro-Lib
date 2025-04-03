#pragma once
#include <cstdint>
#include <functional>
#include <any>
#include <string>
#include <iostream>

//
// Native functions for C++ to call in C#
//

uint8_t* (*GetFunction)(const char* name);

namespace UnityEngine {
	void (*Log)(const char* message);
};

//
// Native functions for C# to call in c++
//

//
// Init function
//

DLLEXPORT void Init(
	uint8_t* (*getFunction)(const char* name)
) {
	GetFunction = getFunction;

	UnityEngine::Log = *(void (**) (const char*))GetFunction("UnityEngine.Log");

	UnityEngine::Log("Hello from C++");
}