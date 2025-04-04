#include "com.h"

void Init(uint8_t* (*getFunction)(const char* name)) {
	GetFunction = getFunction;

	UnityEngine::Log = *(void (**) (const char*))GetFunction("UnityEngine.Log");

	UnityEngine::Log("Hello from C++");
}