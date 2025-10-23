
#include <iostream>

#include "interpolation/InterpSegment.h"

int main() {
	std::cout << "hello world" << std::endl;

	// 插补结果采样周期
	double Ts = 1e-3;
	// 采样细化倍率
	int N = 10;
	// 插补计算周期
	double dt = Ts / N;

	InterpConstraint constraint(5,-5, 10,-8, 30,-40);

	InterpSegment seg0;
	seg0.set_constraint(constraint);

	// 当前插补轨迹
	InterpBoundary boundary;
	std::queue<InterpBoundary> boundaryQueue;
	// 插补轨迹序号
	// 轨迹开始插补周期数, 结束周期数, 开始减速周期数
	int k0 = -1, k1 = 0, kd = 0;
	for (size_t i = 0; i < 1e3; ++i) {
		// 更新插补轨迹
		if (k0 < 0) {
			boundary = boundaryQueue.front();
			boundaryQueue.pop();
			k0 = i;
		}

		// 开始插补

		// 输出插补结果
	}

	return 0;
}
