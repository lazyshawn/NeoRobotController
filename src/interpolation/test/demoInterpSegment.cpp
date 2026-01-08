
#include <iostream>
#include <fstream>
#include <chrono>

#include "interpolation/InterpSegment.h"
#include "interpolation/InterpDispatch.h"

int main() {
	InterpDispatcher dispatcher;
	InterpSignalIn signalIn;
	InterpSignalOut signalOut;
	DispatcherState dispatcherState;
	
	// 初始化
	for (int i=0; i< 1e3; ++i) {
		dispatcher.run_cycle_task(signalIn, signalOut, dispatcherState);
	}

	return 0;
}


int main_() {
	std::cout << "hello world" << std::endl;
	std::string fileName = "interpolation.txt";

	// 约束条件
	InterpConstraint constraint(5,-5, 10,-8, 30,-40);
	// 边界条件
	InterpBoundary boundary(0,10, 1,0, 1,0);

	// 插补轨迹队列
	InterpSegment seg0;
	seg0.set_kinematic_constraint(constraint);
	seg0.add_segment(boundary);
	boundary = InterpBoundary(10,0, 0,2, 0,0);
	seg0.add_segment(boundary);

	// 插补结果
	std::vector<double> q, dq, d2q, d3q, t;
	double dt = seg0.constraint.dt;

	for (size_t i = 0; i < 9e3; ++i) {
		std::vector<double> ans;

		auto t0 = std::chrono::steady_clock::now();
		if (seg0.get_interp_result(ans)) {
			break;
		}
		auto t1 = std::chrono::steady_clock::now();

		// 输出插补结果
		q.push_back(ans[0]);
		dq.push_back(ans[1]);
		d2q.push_back(ans[2]);
		d3q.push_back(ans[3]);
		t.push_back((i + 1)*dt);
	}

	// 插补结果保存到文件
	std::ofstream out(fileName, std::ios::trunc);
	for (size_t i = 0; i < t.size(); ++i) {
		out << t[i] << ", " << q[i] << ", " << dq[i] << ", " << d2q[i] << ", " << d3q[i] << std::endl;
	}

	return 0;
}
