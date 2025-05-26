#ifndef COM_H
#define COM_H
#include <cstdint>
#include <functional>

#include "../macros.h"
#include "../algorithms/algorithms.h"
#include "../types/types.h"

//
// Native functions for C++ to call in C#
//

inline uint8_t* (*GetFunction)(const char* name);

// Interacting with the world
namespace Microsim {
	inline int32_t (*Sensor_i32_ReadValue)(uint32_t sensor);
	inline float (*Sensor_f32_ReadValue)(uint32_t sensor);
	inline v3i (*Sensor_v3i_ReadValue)(uint32_t sensor);

	inline int8_t (*Motor_CurrentThrottle)(uint32_t motor);
	inline void (*Motor_SetThrottle)(uint32_t motor, int8_t throttle);

	inline uint32_t (*Robot_FindComponent)(uint32_t handle, Guid guid);

	inline void (*SimMotorController_Setup)(uint32_t handle, uint32_t robotHandle, void* data);
	inline void (*SimMotorController_UpdateMovement)(uint32_t handle, float dt, RobotPosition pos);
	inline float (*SimMotorController_GetDistanceCovered)(uint32_t handle);
	inline uint32_t (*SimMotorController_GetCurrentState)(uint32_t handle);
	inline float (*SimMotorController_GetTargetDistance)(uint32_t handle);
	inline void (*SimMotorController_SetGyroNull)(uint32_t handle);
	inline void (*SimMotorController_SetRpm)(uint32_t handle, int rpm);
	inline void (*SimMotorController_MoveDistance)(uint32_t handle, float distance);
	inline void (*SimMotorController_RotateToAngle)(uint32_t handle, int wantedAngle);
	inline void (*SimMotorController_RotateDegrees)(uint32_t handle, int degrees);

	inline void (*SimPositionTracker_Setup)(uint32_t handle, uint32_t robotHandle, void* data);
	inline void (*SimPositionTracker_Process)(uint32_t handle, RobotPosition* robotPosition);

	inline uint32_t (*CreateSimulatorAlgorithm)(const char* name);
	inline void (*FreeSimulatorAlgorithm)(uint32_t handle);
};

// Interacting with the engine
namespace UnityEngine {
	inline void (*Log)(const char* message);
	inline void (*Logi)(const char* message, int value);
	inline void (*Logf)(const char* message, float value);
	inline void (*LogV2i)(const char* message, v2i value);
	inline void (*LogV2f)(const char* message, v2f value);

	namespace Debug
	{
		inline void (*DrawRay2D)(v2f start, v2f target, float time, Color color);
		inline void (*DrawRay3D)(v3f start, v3f target, float time, Color color);
		inline void (*DrawLine2D)(v2f a, v2f b, float time, Color color);
		inline void (*DrawLine3D)(v3f a, v3f b, float time, Color color);
	}
};


// Interacting with the plugin
namespace Plugin {
	enum NativeObjectType : uint32_t {
		ObjectDetector,
		RobotController,
		Pathfinder
	};

	typedef std::function<void*()> CreateNativeObject;

	struct NativeObjectFactory {
		const NativeObjectType type;
		const uint32_t idx;
		const char* className;
		const char* name;
	};

	inline void (*RegisterData)(const char* name, const char* version);
	inline void (*RegisterType)(NativeObjectFactory factory);
}

//
// Native functions for C# to call in c++
//

namespace Plugin {
	DLLEXPORT void* Plugin_CreateObject(uint32_t idx);
	DLLEXPORT void Plugin_DeleteObject(void* handle);
}

namespace Microsim {
	DLLEXPORT void RobotController_Setup(IRobotController* ptr, uint32_t robotHandle, void* data);
	DLLEXPORT void RobotController_Loop(IRobotController* ptr, float dtf);

	DLLEXPORT void ObjectDetector_Setup(IObjectDetectorAlgorithm* ptr, uint32_t robotHandle, void* data);
	DLLEXPORT void ObjectDetector_Process(IObjectDetectorAlgorithm* ptr, int* map, v2i mapSize);

	DLLEXPORT void Pathfinder_Setup(IPathfinder* algorithm, uint32_t robotHandle, void* data);
	DLLEXPORT int Pathfinder_Pathfind(IPathfinder* ptr, Map map, RobotPosition position, v2i target, v2i* path);
}

//
// Init function
//

DLLEXPORT void Init(uint8_t* (*getFunction)(const char* name));
DLLEXPORT void Destroy();

#endif //COM_H
