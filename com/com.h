#ifndef COM_H
#define COM_H
#include <cstdint>
#include <functional>
#include <list>

#include "../macros.h"
#include "../algorithms/algorithms.h"

inline std::list<void*> memory;

//
// Native functions for C++ to call in C#
//

inline uint8_t* (*GetFunction)(const char* name);

// Interacting with the world
namespace Microsim {
	struct Object {
		uint32_t handle;
	};

	inline int32_t (*Sensor_ReadValue)(uint32_t sensor);

	inline int8_t (*Motor_CurrentThrottle)(uint32_t motor);
	inline void (*Motor_SetThrottle)(uint32_t motor, int8_t throttle);
};

// Interacting with the engine
namespace UnityEngine {
	inline void (*Log)(const char* message);
};

// Interacting with the plugin
namespace Plugin {
	enum NativeObjectType : uint32_t {
		ObjectDetector
	};

	typedef std::function<void*()> CreateNativeObject;

	struct NativeObjectFactory {
		const NativeObjectType type;
		const uint32_t idx;
		const char* className;
		const char* name;
	};

	inline void (*RegisterType)(NativeObjectFactory factory);
}

//
// Native functions for C# to call in c++
//

namespace Plugin {
	DLLEXPORT void* Plugin_CreateObject(uint32_t size);
	DLLEXPORT void Plugin_DeleteObject(void* handle);
}

namespace Microsim {
	DLLEXPORT void Algorithm_Setup(IAlgorithm* ptr, void* data);

	DLLEXPORT void ObjectDetector_Process(void* ptr, int* map, int mapSize);
}

//
// Init function
//

DLLEXPORT void Init(uint8_t* (*getFunction)(const char* name));

#endif //COM_H
