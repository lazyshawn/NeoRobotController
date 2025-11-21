
#include "interpolation/InterpSegment.h"


InterpConstraint::InterpConstraint(double vmax_, double vmin_, double amax_, double amin_, double jmax_, double jmin_) {
	vmax = vmax_;
	vmin = vmin_;
	amax = amax_;
	amin = amin_;
	jmax = jmax_;
	jmin = jmin_;
}



InterpBoundary::InterpBoundary(double q0_, double q1_, double v0_, double v1_, double a0_, double a1_) {
	qk = q0 = q0_;
	vk = v0 = v0_;
	ak = a0 = a0_;
	q1 = q1_;
	v1 = v1_;
	a1 = a1_;

	jk = 0.0;
}




int InterpSegment::set_kinematic_constraint(const InterpConstraint& constraint_) {
	constraint = constraint_;

	return 0;
}


int InterpSegment::add_segment(const InterpBoundary& boundary) {

	if (boundaryQueue.empty()) {
		cur_q = boundary.q0;
		cur_v = boundary.v0;
		cur_a = boundary.a0;
		cur_j = 0;

		k = 0;
		kb = -1;
	}

	boundaryQueue.push(boundary);

	// 检测边界条件是否超过运动学约束

	return 0;
}

int InterpSegment::pop_front_segment(InterpBoundary& boundary) {
	if (boundaryQueue.empty())
		return 1;

	boundary = boundaryQueue.front();

	boundaryQueue.pop();

	return 0;
}

int InterpSegment::get_interp_result(std::vector<double>& result) {

	// 插补队列为空
	if (boundaryQueue.empty()) {
		return 1;
	}

	double qk1, vk1, ak1, jk1;
	// 当前轨迹的目标状态
	int sign = boundaryQueue.front().q1 - boundaryQueue.front().q0 > 0 ? 1 : -1;
	double q0 = sign * boundaryQueue.front().q0;
	double q1 = sign * boundaryQueue.front().q1;
	double v1 = sign * boundaryQueue.front().v1;
	double a1 = sign * boundaryQueue.front().a1;
	// 当前约束
	double vmin = (sign + 1) / 2 * constraint.vmin + (sign - 1) / 2 * constraint.vmax;
	double vmax = (sign + 1) / 2 * constraint.vmax + (sign - 1) / 2 * constraint.vmin;
	double amin = (sign + 1) / 2 * constraint.amin + (sign - 1) / 2 * constraint.amax;
	double amax = (sign + 1) / 2 * constraint.amax + (sign - 1) / 2 * constraint.amin;
	double jmin = (sign + 1) / 2 * constraint.jmin + (sign - 1) / 2 * constraint.jmax;
	double jmax = (sign + 1) / 2 * constraint.jmax + (sign - 1) / 2 * constraint.jmin;
	double dt = constraint.dt;
	// 插补完成
	bool interpFinish = false;

	// 计算
	for (size_t j = 0; j < constraint.N; ++j) {
		k++;
		// 当前计算周期内的初始值
		double qk = sign * cur_q, vk = sign * cur_v, ak = sign * cur_a, jk = sign * cur_j;
		// 当前计算周期内的目标值

		// 开始插补
		// 减速阶段检测
		if (kb < 0) {
			Tj2a = (amin - ak) / jmin;
			Tj2b = (a1 - amin) / jmax;
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
				kb = k;
				printf("decel at: %d. %f, %f, %f\n", k, qk, hk, q1);
			}

		}

		// 规划减速阶段
		int accelFlag = 8;
		if (kb >= 0) {
			if (k < Tj2a / dt + kb) {
				jk1 = jmin;
			}
			else if (k < (Td - Tj2b) / dt + kb) {
				jk1 = 0;
			}
			else if (k < Td / dt + kb) {
				jk1 = jmax;
				// 提前到位(加速度突变)
				if (std::fabs(qk - q1) < constraint.inPlacePos && std::fabs(vk - v1) < constraint.inPlaceVel) {
					interpFinish = true;
				}
			}
			else if (k >= Td / dt + kb) {
				jk1 = jk;
				// 减速阶段结束(位置偏差)
				if (k > 0 && k % constraint.N == 0) {
					interpFinish = true;
				}
			}

			// 所有插补轨迹结束,输出目标点
			if (interpFinish && boundaryQueue.size() == 1) {
				jk1 = 0;
				qk1 = q1;
				vk1 = v1;
				ak1 = a1;
				accelFlag = 15;
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
				accelFlag = accelFlag + 4;
			}
		}

		// 更新当前状态
		if ((accelFlag >> 2) % 2 == 0) {
			ak1 = ak + dt * (jk + jk1) / 2;
		}
		if ((accelFlag >> 1) % 2 == 0) {
			vk1 = vk + dt * (ak + ak1) / 2;
		}
		if (accelFlag % 2 == 0) {
			qk1 = qk + dt * (vk + vk1) / 2;
			// 位置校正
			//if (q1 < qk1) {
			//	qk1 = qk1 + 0.1 * (q1 - qk1);
			//}
		}

		// 当前状态
		cur_q = sign * qk1, cur_v = sign * vk1, cur_a = sign * ak1, cur_j = sign * jk1;
	}

	result = std::vector<double>({ cur_q, cur_v, cur_a, cur_j });
	// 当前段插补完成
	if (interpFinish) {
		kb = -1;
		boundaryQueue.pop();
	}

	return 0;
}

