# Unity Micro Lib

This repository the communication layer for C# & C++. And also some algorithms used for our mouse.

## Repository Legenda

- `/com/*` The main communication layer between C# & C++.
- `/algorithms/*` Contains all the interfaces for the algorithms, also some algorithms implemented in the simulator like the MotorController and the PositionTracker.
- `/microsim/*` Some objects that help interacting with the simulator, like sensors & object handles.
- `/types/*` Data types like v2i, v2f, Map & more. Please note that some types like v2i are partially implemented and only have what we needed whilst making this project.
- `/examples/*` All the algorithms (except the motorcontroller and positiontracker, see the [Arduino micro lib](https://github.com/Team-MicroMouse/Arduino-Micro-Lib)).

## Adding an algorithm.

We added export types for 3 algorithms, `RobotController, ObjectDetector and PathFinder`. These also have tests in the simulator. If you want to add a new algorihm, you
can add a folder in the `examples` directory, then register it in `com.cpp` in the `Init` function under the comment `Registering Objects`. Then when you want to use it in
the simulator, go to one of the tests and change the algorithm name. If you want to use it in the mouse itself, you can change one of the fields in the Robot2 prefab.

## Build instructions

Make sure you have the [simulator](https://github.com/Team-MicroMouse/simulatie) downloaded and set up!

### Windows

If you use Visual Studio, I'm sorry for you, and congratulations, its the easiest for you! Just make sure to use cmake to build the Visual Studio project files and whabam! Your dll should be ready to
roll. To test it, put the dll into the simulator's Assets directory named `Unity_Link_Test.dll`.

If you use CLion or any other IDE on windows, please make sure to install Build Tools for Visual Studio [here](https://visualstudio.microsoft.com/downloads/). Then in CLion, make sure to select it as
your build tool, and use it to build the project, then put the dll into the simulator's Assets directory named `Unity_Link_Test.dll`.

### Linux

Build your build config with cmake for your preferred toolchain, then compile the linked library. Lastly, put the dll into the simulator's Assets directory named `libUnity_Link_Test.so`.

### MacOS

We aren't sure if it works for MacOS, and if it doesn't, you're on your own, we can't help you! With that out of the way, build your build config with cmake for your preferred toolchain, then compile the library. 
Lastly, put the dll into the simulator's Assets directory named `libUnity_Link_Test.dylib`.
