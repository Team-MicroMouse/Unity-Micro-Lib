#ifndef ALGORITHMS_H
#define ALGORITHMS_H

struct IAlgorithm {
	virtual ~IAlgorithm() = default;
	virtual void Setup(void*) = 0;
};

struct IObjectDetectorAlgorithm : IAlgorithm {
	virtual void Process(int* map, int mapSize) = 0;
};

struct TestObjectDetector : IObjectDetectorAlgorithm {
	void Setup(void*) override;
	void Process(int* map, int mapSize) override;
};

#endif //ALGORITHMS_H
