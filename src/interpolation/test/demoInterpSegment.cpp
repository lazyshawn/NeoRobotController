
#include <iostream>

#include "interpolation/InterpSegment.h"

int main() {
	std::cout << "hello world" << std::endl;

	// 插补结果采样周期
	double Ts = 1e-3;
	// 采样细化倍率
	int N = 1;
	// 插补计算周期
	double dt = Ts / N;

	InterpConstraint constraint(5,-5, 10,-8, 30,-40);
	InterpBoundary boundary(0,10, 1,0, 1,0);

	InterpSegment seg0;
	seg0.set_constraint(constraint);
	seg0.add_segment(boundary);

	// 插补轨迹序号
	// 轨迹开始插补周期数, 结束周期数, 开始减速周期数
	int k0 = -1, k1 = 0, kd = 0;
	double qk1 = 0.0, vk1 = 0.0, ak1 = 0.0, jk1 = 0.0;
	double q1 = 0.0, v1 = 0.0, a1 = 0.0, j1 = 0.0;
	bool decelPhase = false, interpFinish = false;
	for (size_t i = 0; i < 1e3; ++i) {
		// 更新插补轨迹
		if (k0 < 0) {
			seg0.pop_front_segment(boundary);
			// 当前轨迹段开始插补周期
			k0 = i;
			qk1 = boundary.q0, vk1 = boundary.v0, ak1 = boundary.a0, jk1 = 0;
			q1 = boundary.q1, v1 = boundary.v1, a1 = boundary.a1, j1 = 0;
		}

		// 当前状态
		double qk = qk1, vk = vk1, ak = ak1, jk = jk1;
		// 当前约束
		double vmin = seg0.constraint.vmin;
		double amin = seg0.constraint.amin;
		double jmin = seg0.constraint.jmin;
		double vmax = seg0.constraint.vmax;
		double amax = seg0.constraint.amax;
		double jmax = seg0.constraint.jmax;

		// 开始插补
		double Td = 0.0, Tj2a = 0.0, Tj2b = 0.0;
		int kb = -1;
		// 减速阶段检测
		if (!decelPhase) {
			double Tj2a = (amin - ak) / jmin;
			double Tj2b = (a1 - amin) / jmax;
			Td = (v1 - vk) / amin + Tj2a * (amin - ak) / (2 * amin) + Tj2b * (amin - a1) / (2 * amin);

			// 不能达到最小速度
			if (vk > v1 && Td < Tj2a + Tj2b) {
				double det = std::sqrt((jmax - jmin)*(ak*ak*jmax - jmin * (a1*a1 + 2 * jmax*(vk - v1))));
				Tj2a = -ak / jmin + det / (jmin*(jmin - jmax));
				Tj2a = a1 / jmax + det / (jmax*(jmax - jmin));
				Td = Tj2a + Tj2b;
			}

			// 计算减速段距离
			double hk = ak * Td*Td / 2 + (jmin*Tj2a*(3 * Td*Td - 3 * Td*Tj2a + Tj2a * Tj2a) + jmax * Tj2b*Tj2b*Tj2b) / 6 + Td * vk;

			// 进入减速阶段
			if (qk + hk >= q1) {
				decelPhase = true;
				kb = i;
				//kd1 = np.ceil(Tj2a / dt) + kb;
				//kd2 = np.ceil((Td - Tj2b) / dt) + kb;
				//ke = np.ceil(Td / dt) + kb;
				//print(f"decel phase begin: {kb}, {Td}, {Tj2a}, {Tj2b}, {kd1}, {kd2}, {ke}");
			}

		}

		// 规划减速阶段
		int accelFlag = 8;
		if (decelPhase) {
			if (i < Tj2a / dt + kb) {
				jk1 = jmin;
			}
			else if (i < (Td - Tj2b) / dt + kb) {
				jk1 = 0;
			}
			else if (i < Td / dt + kb) {
				jk1 = jmax;
			}
			else if (i >= Td / dt + kb) {
				jk1 = 0;
				if (i > 0 && i % N == 0) {
					interpFinish = true;
				}
			}
		}
		// 规划加速段
		else {
			// 实际可达最大速度
			double vlim = vk - ak * ak / (2 * jmin);

			// 计算当前加加速度
			if (vlim < vmax && ak < amax) {
				jk1 = jmax;
			}
			else if (vlim < vmax && ak >= amax) {
				jk1 = 0;
			}
			else if (vlim >= vmax && ak > 0) {
				jk1 = jmin;
			}
			// 进入匀速阶段
			else if (vlim >= vmax && ak <= 0) {
				jk1 = 0;
				ak1 = 0;
				//accelFlag = accelFlag + 4;
			}
		}

		// 更新当前状态
		

		// 输出插补结果

	}

	return 0;
}
