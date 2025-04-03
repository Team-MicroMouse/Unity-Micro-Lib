#pragma once

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>

extern "C" {
	struct int2 {int x; int y;};
	struct float2 { float x; float y; };
	
	struct RobotPosition {
		int2 position;
		int angle;
	};
}