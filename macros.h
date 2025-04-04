//
// Created by Jeroen on 3-4-2025.
//

#ifndef MACROS_H
#define MACROS_H

#ifdef _WIN32
#define DLLEXPORT extern "C" __declspec(dllexport)
#else
#define DLLEXPORT extern "C"
#endif

#endif //MACROS_H
