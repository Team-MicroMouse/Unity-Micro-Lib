#ifndef ALGORITHMS_H
#define ALGORITHMS_H

struct  IAlgorithm {
	virtual void Setup(void*) = 0;
	virtual ~IAlgorithm() {}
};

struct IObjectDetectorAlgorithm : IAlgorithm {
	virtual void Process(int* map, int mapSize) = 0;
};

struct TestObjectDetector : IObjectDetectorAlgorithm {
	void Setup(void*) override;
	void Process(int* map, int mapSize) override;
};

#endif //ALGORITHMS_H
