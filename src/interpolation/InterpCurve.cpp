
#include "interpolation/InterpCurve.h"

DoubleSCurve::DoubleSCurve() {
	vmin = -vmax;
	amin = -amax;
	jmin = -jmax;
}

int DoubleSCurve::set_condition(double begPos, double endPos, double begVel, double endVel) {
	q0 = begPos;
	q1 = endPos;
	v0 = begVel;
	v1 = endVel;

	sign = (q0 > q1) ? -1 : 1;
	q0 *= sign;
	q1 *= sign;
	v0 *= sign;
	v1 *= sign;
	vmax = (sign + 1) / 2 * vmax + (sign - 1) / 2 * vmin;
	vmin = (sign + 1) / 2 * vmin + (sign - 1) / 2 * vmax;
	amax = (sign + 1) / 2 * amax + (sign - 1) / 2 * amin;
	amin = (sign + 1) / 2 * amin + (sign - 1) / 2 * amax;
	jmax = (sign + 1) / 2 * jmax + (sign - 1) / 2 * jmin;
	jmin = (sign + 1) / 2 * jmin + (sign - 1) / 2 * jmax;

	return 0;
}

int DoubleSCurve::set_reserve_time(double time) {
	reserveTime = time;
	return 0;
}

int DoubleSCurve::plan() {
	double Tjs1 = std::sqrt(std::fabs(v1 - v0) / jmax);
	double Tjs2 = amax / jmax;
	double Tjs = Tjs1;
	
	// - 轨迹合法性检测
	bool valid = true;
	if (Tjs1 < Tjs2) {
		if (q1 - q0 < Tjs * (v0 + v1))
			valid = false;
	}
	else {
		Tjs = Tjs2;
		if (q1 - q0 < (v0 + v1) / 2 * (Tjs + std::fabs(v1 - v0) / jmax))
			valid = false;
	}
	// 规划失败，无法在给定约束下通过双S曲线到达目标位置
	if (!valid)
		return 1;

	// - 计算各阶段时间
	// Case 1: vlim = vmax
	// 加速阶段达到最大速度
	if ((vmax-v0)*jmax < amax*amax) {
		Tj1 = std::sqrt(std::fabs(vmax - v0) / jmax);
		Ta = 2 * Tj1;
	}
	else {
		Tj1 = amax / jmax;
		Ta = Tj1 + (vmax - v0) / amax;
	}
	// 减速阶段达到最大速度
	if ((vmax - v1)*jmax < amax*amax) {
		Tj2 = std::sqrt(std::fabs(vmax - v1) / jmax);
		Td = 2 * Tj2;
	}
	else {
		Tj2 = amax / jmax;
		Td = Tj2 + (vmax - v1) / amax;
	}
	Tv = (q1 - q0) / vmax - Ta / 2 * (1 + v0 / vmax) - Td / 2 * (1 + v1 / vmax);

	// Case 2: vlim < vmax
	if (Tv < 0) {
		Tv = Ta = Td = 0.0;

		double gama = 1.0, Tj = amax / jmax;
		for (int i = 0; i < 10; ++i) {
			double alim = gama * amax;

			// Case 2.1: alim = amax
			double det = std::pow(alim, 4.0) / jmax / jmax + 2 * (v0*v0 + v1 * v1) + alim * (4 * (q1 - q0) - 2 * alim / jmax * (v0 + v1));
			Tj = Tj1 = Tj2 = alim / jmax;
			Ta = (alim*alim / jmax - 2 * v0 + std::sqrt(det)) / (2 * alim);
			Td = (alim*alim / jmax - 2 * v1 + std::sqrt(det)) / (2 * alim);

			// this case is possible when v0 > v1
			if (Ta < 0) {
				Ta = Tj1 = 0.0;
				Td = 2 * (q1 - q0) / (v1 + v0);
				Tj2 = (jmax * (q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / (jmax*(v1 + v0));
			}
			// this case is possible when v1 > v0
			else if (Td < 0) {
				Td = Tj2 = 0;
				Ta = 2 * (q1 - q0) / (v1 + v0);
				Tj1 = (jmax * (q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) - (v1 + v0)*(v1 + v0)*(v1 - v0)))) / (jmax*(v1 + v0));
			}
			// Case 2.2: alim < amax, 继续减小加速度限制
			else if (Ta < 2 * Tj || Td < 2 * Tj) {
				gama *= 0.8;
			}
		}
	}

	// - 计算最大速度、加速度
	alima = jmax * Tj1;
	alimd = -jmax * Tj2;
	vlim = v0 + (Ta - Tj1)*alima;
	T = Ta + Tv + Td;

	return 0;
}

double DoubleSCurve::get_duration() {
	return T;
}

double DoubleSCurve::get_Ta() {
	return Ta;
}

double DoubleSCurve::get_Td() {
	return Td;
}

bool DoubleSCurve::done() {
	return doneFlag;
}

double DoubleSCurve::get_offset() {
	return offset;
}

double DoubleSCurve::get_pos(double t) {
	
	// 时间缩放
	t = t * scale + offset;

	// 插补完成标志
	if (t + reserveTime > T) {
		t = T - reserveTime;
		doneFlag = true;
	}
	else {
		doneFlag = false;
	}

	double q = q0, v = v0, a = 0.0, j = 0.0;
	double jmin = -jmax;
	if (t < 0) {
		q = q0;
		v = v0;
	}
	// 加速阶段
	else if (t < Tj1) {
		q = q0 + v0 * t + jmax * t*t*t / 6;
		v = v0 + jmax * t*t / 2;
		a = jmax * t;
		j = jmax;
	}
	else if (t < Ta - Tj1) {
		q = q0 + v0 * t + alima / 6 * (3 * t*t - 3 * Tj1*t + Tj1 * Tj1);
		v = v0 + alima * (t - Tj1 / 2);
		a = alima;
		j = 0;
	}
	else if (t < Ta) {
		double curT = Ta - t;
		q = q0 + (vlim + v0) * Ta / 2 - vlim * curT - jmin * curT*curT*curT / 6;
		v = vlim + jmin * curT*curT / 2;
		a = -jmin * curT;
		j = jmin;
	}
	// 匀速阶段
	else if (t < Ta + Tv) {
		q = q0 + (vlim + v0) * Ta / 2 + vlim * (t - Ta);
		v = vlim;
		a = 0;
		j = 0;
	}
	// 减速阶段
	else if (t < T - Td + Tj2) {
		double curT = t - T + Td;
		q = q1 - (vlim + v1)*Td / 2 + vlim * curT - jmax * curT*curT*curT / 6;
		v = vlim - jmax * curT*curT / 2;
		a = -jmax * curT;
		j = jmin;
	}
	else if (t < T - Tj2) {
		double curT = t - T + Td;
		q = q1 - (vlim + v1)*Td / 2 + vlim * curT + alimd / 6 * (3 * curT*curT - 3 * Tj2*curT + Tj2 * Tj2);
		v = vlim + alimd * (curT - Tj2 / 2);
		a = alimd;
		j = 0;
	}
	else if (t < T) {
		double curT = T - t;
		q = q1 - v1 * curT - jmax * curT*curT*curT / 6;
		v = v1 + jmax * curT*curT / 2;
		a = -jmax * curT;
		j = jmax;
	}
	// 超出规划时间
	else {
		q = q1;
		v = v1;
	}

	// 插补完成标志
	//doneFlag = t + reserveTime > T;

	q *= sign;
	v *= sign;
	a *= sign;
	j *= sign;

	vp = v;

	return q;
}

int DoubleSCurve::displacement(double dt, double k) {
	offset = dt;
	scale = k;
	return 0;
}

double DoubleSCurve::get_remain_dist(double dt) {
	int num = T / dt;
	double endT = num * dt;
	return q1 - get_pos(endT);
}
