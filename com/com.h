#ifndef COM_H
#define COM_H
#include <cstdint>
#include "../macros.h"

//
// Native functions for C++ to call in C#
//

inline uint8_t* (*GetFunction)(const char* name);

namespace UnityEngine {
	inline void (*Log)(const char* message);
};

namespace Microsim {
	inline int32_t (*Sensor_ReadValue)(int32_t sensor);
	inline int8_t (*Motor_CurrentThrottle)(int32_t motor);
	inline void (*Motor_SetThrottle)(int32_t motor, int8_t throttle);
};

//
// Native functions for C# to call in c++
//

//
// Init function
//

DLLEXPORT void Init(uint8_t* (*getFunction)(const char* name));

#endif //COM_H
