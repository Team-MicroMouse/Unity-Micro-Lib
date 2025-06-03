//
// Created by Jeroen on 3-4-2025.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <cstdint>
#include "../types/types.h"

namespace Microsim {
	struct Robot;

	struct Object {
		uint32_t handle;
	};

	/* Interfaces */

	struct IExtractableData {
	};

	/* Sensors & Motors */

	struct Sensor_i32 : Object {
		int32_t ReadValue() const;
	};

	struct Sensor_f32 : Object {
		float ReadValue() const;
	};

	struct Sensor_v3i : Object {
		  v3i ReadValue() const;
	};

	struct Motor : Object {
		int8_t CurrentThrottle() const;
		void SetThrottle(int8_t throttle) const;
	};

	struct IFindableComponent : Object {
		Sensor_i32 ToSensor_i32() const;
		Sensor_f32 ToSensor_f32() const;
		Sensor_v3i ToSensor_v3i() const;
		Motor ToMotor() const;
	};

	/* Data Types */

	struct ComponentReference {
		Guid serializableGuid;
	};

	/* Components */

	struct Robot : Object {
		IFindableComponent FindComponent(Guid serializableGuid) const;
	};
}

#endif //INTERFACE_H
