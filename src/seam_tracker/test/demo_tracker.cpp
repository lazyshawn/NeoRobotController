
#include <iostream>
#include <random>
#include <chrono>

#include "zmotion_tracker_interface.h"

const int sampleNum = 100;
double sampleVec[sampleNum];
double fineVec[sampleNum];

int main() {
	std::cout << "hello world" << std::endl;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine engine(seed);
	std::normal_distribution<double> distribute(100, 5);

	// 滤波初始化
	reset_mw_filter(0, sampleVec);

	// 开始滤波
	//for (size_t i = 0; i < sampleNum; ++i) {
	//	sampleVec[0] = i + 1;
	//	sampleVec[i + 1] = distribute(engine);
	//	fineVec[0] = i + 1;
	//	fineVec[i + 1] = moving_window_filter(0, sampleVec[i + 1]);
	//	std::cout << sampleVec[i + 1] << ", " << fineVec[i + 1] << std::endl;
	//}

	for (size_t i = 0; i < sampleNum; ++i) {
		sampleVec[i] = 0;
	}
	for (size_t i = 0; i < 20; ++i) {
		double ret = moving_window_filter(0, sampleVec[i]);
		std::cout << ret << std::endl;
	}

	printf("Press <Enter> to exit.\n");
	getchar();
}
