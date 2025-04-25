//
// Created by Jeroen on 22-4-2025.
//

#ifndef SIMPLEOBJECTDETECTOR_H
#define SIMPLEOBJECTDETECTOR_H
#include "../../algorithms/algorithms.h"


class SimpleObjectDetector : IObjectDetectorAlgorithm {
    void Setup(Microsim::Robot robot, void *) override;
    void Process(int *map, v2i mapSize) override;
};


#endif //SIMPLEOBJECTDETECTOR_H
