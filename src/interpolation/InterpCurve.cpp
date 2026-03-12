
#include "interpolation/InterpCurve.h"

// --- 辅助函数
// 解二次方程
int solve_quadratic_eqution(double *k, double *ans) {
	double c = k[0], b = k[1], a = k[2];

	double det = b * b - 4 * a*c;
	if (det < 0)
		return -1;

	ans[0] = (-b + sqrt(det)) / (2 * a);
	ans[1] = (-b - sqrt(det)) / (2 * a);
	
	// 从小到大排序
	if (ans[1] < ans[0]) {
		double tmp = ans[1];
		ans[1] = ans[0];
		ans[0] = ans[1];
	}

	return 0;
}

// 解三次方程, 盛金公式
double solve_cubic_eqution(double *k, double *ans) {
	double d = k[0], c = k[1], b = k[2], a = k[3];
	if (a < 0) {
		a *= -1;
		b *= -1;
		c *= -1;
		d *= -1;
	}

	// 重根判别式
	double A = b * b - 3 * a*c;
	double B = b * c - 9 * a*d;
	double C = c * c - 3 * b*d;

	// 总判别式
	double Det = B * B - 4 * A*C;
	double epsillon = 1e-6;

	// 公式1: 三重实根
	if (fabs(A) < epsillon && fabs(B) < epsillon) {
		ans[0] = ans[1] = ans[2] = (-b - c - 3 * d) / (3 * a + b + c);
	}
	// 公式3: 三个实根，其中一个两重实根
	else if (fabs(Det) < epsillon) {
		double K = B / A;
		double sol = -b / a + K;
		ans[0] = sol;

		sol = -K / 2;
		ans[1] = ans[2] = sol;
	}
	else {
		// 公式 4: 三个不同实根
		if (Det < 0) {
			double T = (2 * A*b - 3 * a*B) / (2 * sqrt(A*A*A));
			double q = acos(T);

			double sol = (-b - 2 * sqrt(A)*cos(q / 3)) / (3 * a);
			ans[0] = sol;

			sol = (-b + sqrt(A)*(cos(q / 3) + sqrt(3)*sin(q / 3))) / (3 * a);
			ans[1] = sol;

			sol = (-b + sqrt(A)*(cos(q / 3) - sqrt(3)*sin(q / 3))) / (3 * a);
			ans[2] = sol;
		}
		// 公式2: 一个实根，一对共轭虚根
		else if (Det > 0) {
			double Y1 = A * b + 3 * a*(-B + sqrt(B*B - 4 * A*C)) / 2;
			double Y2 = A * b + 3 * a*(-B - sqrt(B*B - 4 * A*C)) / 2;
			double sol = (-b - cbrt(Y1) - cbrt(Y2)) / (3 * a);
			ans[0] = ans[1] = ans[2] = sol;
		}
	}

	// 从小到大排序
	for (int i = 0; i < 3; ++i) {
		for (int j = i + 1; j < 3; ++j) {
			// 后者更小，交换顺序
			if (ans[j] < ans[i]) {
				double tmp = ans[i];
				ans[i] = ans[j];
				ans[j] = tmp;
			}
		}
	}

	// 返回最小正实根
	for (int i = 0; i < 3; ++i) {
		if (ans[i] > 0)
			return ans[i];
	}
}


/***********************************************************************
 *                        DoubleSCurve                                 *
 ***********************************************************************/

DoubleSCurve::DoubleSCurve() {
	vmin = -vmax;
	amin = -amax;
	jmin = -jmax;
}

int DoubleSCurve::calc_plan_param() {

	// 速度、加速度上确界
	alima = jmax * Tj1;
	alimd = -jmax * Tj2;
	vlim = v0 + (Ta - Tj1)*alima;
	T = Ta + Tv + Td;

	// 不同阶段的运动位移
	double t = Tj1;
	s1 = v0 * t + jmax * t*t*t / 6;
	t = Ta - Tj1;
	s2 = v0 * t + alima / 6 * (3 * t*t - 3 * Tj1*t + Tj1 * Tj1);
	s3 = (vlim + v0) * Ta / 2;
	t = Tv;
	s4 = (vlim + v0) * Ta / 2 + vlim * t;
	t = Tj2;
	s5 = q1 - q0 - (vlim + v1)*Td / 2 + vlim * t - jmax * t*t*t / 6;
	t = Td - Tj2;
	s6 = q1 - q0 - (vlim + v1)*Td / 2 + vlim * t + alimd / 6 * (3 * t * t - 3 * Tj2 * t + Tj2 * Tj2);

	return 0;
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

int DoubleSCurve::set_constraint(double maxVel, double maxAcc) {
	vmax = fabs(maxVel);
	amax = fabs(maxAcc);

	vmin = -vmax;
	amin = -amax;
	return 0;
}

int DoubleSCurve::set_reserve_time(double time) {
	reserveTime = time;
	return 0;
}

int DoubleSCurve::plan() {
	// 不考虑加速度限制，两段加速到达目标点时，加加速时间Tjs1
	double Tjs1 = std::sqrt(std::fabs(v1 - v0) / jmax);
	// 到达最大加速度时间，到达后需要匀加速阶段，即三段加速到达目标点，此时加加速时间Tjs2
	double Tjs2 = amax / jmax;
	double Tjs = Tjs1;
	
	// - 轨迹合法性检测: 单个加速/减速阶段即达到目标点速度，验证最短轨迹长度
	bool valid = true;
	double minDis = 0.0;
	// 两段加速的最短运动距离
	if (Tjs1 < Tjs2) {
		minDis = Tjs * (v0 + v1);
	}
	// 三段加速的最短运动距离
	else {
		Tjs = Tjs2;
		minDis = (v0 + v1) / 2 * (Tjs + std::fabs(v1 - v0) / jmax);
	}
	// 规划失败，无法在给定约束下通过双S曲线到达目标位置，速度规划时需要规避这种情况
	if (q1 - q0 < minDis) {
		valid = false;
		return 1;
	}

	// - 计算各阶段时间
	// Case 1: vlim = vmax
	// 加速阶段达到最大加速度
	if ((vmax-v0)*jmax < amax*amax) {
		Tj1 = std::sqrt(std::fabs(vmax - v0) / jmax);
		Ta = 2 * Tj1;
	}
	else {
		Tj1 = amax / jmax;
		Ta = Tj1 + (vmax - v0) / amax;
	}
	// 减速阶段达到最大加速度
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
		valid = false;
		double low = 0.0, upp = 1.0;
		for (int i = 0; i < 10; ++i) {
			double mid = (low + upp) / 2.0;
			// 当前加速度限制可以规划，二分区间上移
			bool moveLow = true;
			// 迭代解
			double tj, tj1, tj2, ta, td, alim = mid * amax;

			// Case 2.1: alim = amax
			double det = std::pow(alim, 4.0) / jmax / jmax + 2 * (v0*v0 + v1 * v1) + alim * (4 * (q1 - q0) - 2 * alim / jmax * (v0 + v1));
			tj = tj1 = tj2 = alim / jmax;
			ta = (alim*alim / jmax - 2 * v0 + std::sqrt(det)) / (2 * alim);
			td = (alim*alim / jmax - 2 * v1 + std::sqrt(det)) / (2 * alim);

			// 仅需一个加速阶段，可以规划 (发生在v0 > v1)
			if (ta < 0) {
				ta = tj1 = 0.0;
				td = 2 * (q1 - q0) / (v1 + v0);
				tj2 = (jmax * (q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / (jmax*(v1 + v0));
			}
			// 仅需一个减速阶段，可以规划 (发生在v0 < v1)
			else if (td < 0) {
				td = tj2 = 0;
				ta = 2 * (q1 - q0) / (v1 + v0);
				tj1 = (jmax * (q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) - (v1 + v0)*(v1 + v0)*(v1 - v0)))) / (jmax*(v1 + v0));
			}
			// Case 2.2: alim < amax, 继续减小加速度限制
			else if (ta < 2 * tj || td < 2 * tj) {
				moveLow = false;
			}
			// 两个阶段，两段都达到最大加速度时可以规划，此时Tj段时间相同

			// 当前区间终点可以规划
			if (moveLow) {
				valid = true;
				// 迭代更新
				Ta = ta;
				Tj1 = tj1;
				Td = td;
				Tj2 = tj2;
				low = mid;

				// 第一次迭代就成功则直接返回，当前即为封闭解
				if (i == 0)
					break;
			}
			else {
				upp = mid;
			}
		}
	}

	// 规划失败，无法找到合适的加速度
	if (!valid)
		return 2;

	// - 计算最大速度、加速度
	calc_plan_param();

	return 0;
}

int DoubleSCurve::plan_by_duration(double Tall, double Tacc, double Tjerk) {
	double alpha = Tacc / Tall;
	double beta = Tjerk / Tacc;
	double h = q1 - q0;

	T = Tall;
	Ta = Td = Tacc;
	Tj1 = Tj2 = Tjerk;
	Tv = Tall - Ta - Td;
	
	double denom = (1 - alpha) * T;
	vmax = h / denom;
	denom *= alpha * (1 - beta) * T;
	amax = h / denom;
	denom *= alpha * beta * T;
	jmax = h / denom;

	// - 计算最大速度、加速度
	calc_plan_param();

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

double DoubleSCurve::get_Tv() {
	return Tv;
}

double DoubleSCurve::get_vp() {
	return vp;
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

double DoubleSCurve::get_remain_time(double dt) {
	int num = T / dt;
	double endT = num * dt;
	return T - endT;
}

double DoubleSCurve::get_max_speed(double ds) {

	// 速度冗余，防止计算误差导致规划时最大速度超出限制
	double margin = 1e-9;
	double Tjk, Tacc;
	// 加速到最大速度过程中达到最大加速度
	if ((vmax - v0)*jmax < amax*amax) {
		Tjk = std::sqrt(std::fabs(vmax - v0) / jmax);
		Tacc = 2 * Tjk;
	}
	else {
		Tjk = amax / jmax;
		Tacc = Tjk + (vmax - v0) / amax;
	}
	// 加速到最大速度需要的临界距离
	double dsCmax = Tacc * (v0 + vmax) / 2;

	double ans = 0.0;
	// 1. 加速距离内能达到最大速度
	if (ds > dsCmax) {
		ans = vmax;
	}
	// 2. 距离不够加速到最大速度
	else {
		// 无法达到最大速度时，加速到目标距离的临界情况：两段加速刚好达到最大加速度
		double dsC2 = (2 * v0*jmax + amax * amax) * amax / (jmax*jmax);
		double sol[3] = { 0.0 };
		int solNum = 3;

		// 2.1. 需要三段加速到达目标距离
		if (ds > dsC2) {
			Tjk = amax / jmax;
			double coeff[3] = { Tjk / 2 * v0 - v0 * v0 / (2 * amax), Tjk / 2, 1 / (2 * amax) };
			solve_quadratic_eqution(coeff, sol);
		}
		// 2.2. 两段加速到达目标距离
		else {
			double coeff[4] = { -v0 * v0 * v0 - ds * ds * jmax, -v0 * v0, v0, 1.0 };
			solNum = 2;
			solve_cubic_eqution(coeff, sol);
		}

		// 最小正解
		for (int i = 0; i < solNum; ++i) {
			if (sol[i] > 0) {
				ans = sol[i];
				break;
			}
		}
	}

	return ans - margin;
}

double DoubleSCurve::calc_time_PiTPe(double ds) {
	// 当前运动距离
	double s = q1 - q0 - ds;


	double sol[3] = { 0.0 }, t0 = 0.0;
	double solNum = 2, solShift = 0.0;
	if (s < 0) {
		sol[0] = T;
		solNum = 1;
	}
	else if (s < s1) {
		double coeff[4] = { -s, v0, 0, jmax / 6 };
		solve_cubic_eqution(coeff, sol);
		solNum = 3;
	}
	else if (s < s2) {
		double coeff[3] = { alima*Tj1*Tj1 / 6 - s, v0 - alima * Tj1 / 2, alima / 2 };
		solve_quadratic_eqution(coeff, sol);
	}
	else if (s < s3) {
		double coeff[4] = { s - (vlim + v0)*Ta / 2, vlim, 0, jmin / 6 };
		solve_cubic_eqution(coeff, sol);
		solNum = 3;
		solShift = -Ta;
	}
	else if (s < s4) {
		sol[0] = (s - (vlim + v0)*Ta / 2) / vlim + Ta;
		solNum = 1;
	}
	else if (s < s5) {
		double coeff[4] = { s - (vlim + v1)*Td / 2, -vlim, 0, jmax / 6 };
		solve_cubic_eqution(coeff, sol);
		solNum = 3;
		solShift = T - Td;
	}
	else if (s < s6) {
		double coeff[3] = { alimd*Tj2*Tj2 / 6 - (vlim + v1)*Tj2 / 2 - s, vlim - alimd * Tj2 / 2, alimd / 2 };
		solve_quadratic_eqution(coeff, sol);
		solShift = T - Td;
	}
	else if (s < q1 - q0) {
		double coeff[4] = { s, v1, 0, jmax / 6 };
		solve_cubic_eqution(coeff, sol);
		solNum = 3;
		solShift = -T;
	}
	else {
		sol[0] = 0;
		solNum = 1;
	}

	// 最小正解
	double ans = 0.0;
	for (int i = 0; i < solNum; ++i) {
		if (sol[i] > 0) {
			ans = sol[i];
			break;
		}
	}
	ans = T - std::fabs(ans + solShift);

	return ans;
}

