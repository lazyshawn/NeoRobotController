
#include "InterpCurve.h"
#include <iostream>

// --- 辅助函数
// 解二次方程: ax^2 + bx + c = 0
// 输入: k[0]=c, k[1]=b, k[2]=a
// 输出: ans[0], ans[1] (从小到大排序)
// 返回值: 0-成功, -1-无实数解, -2-非二次方程(a=0)
static int solve_quadratic_equation(double *k, double *ans) {
	double c = k[0], b = k[1], a = k[2];
	
	// 处理非二次方程的情况
	if (fabs(a) < 1e-10) {
		// 一次方程: bx + c = 0
		if (fabs(b) < 1e-10) {
			// 无解或无穷多解
			return -1;
		}
		ans[0] = -c / b;
		ans[1] = ans[0];
		return 0;
	}

	// 计算判别式
	double det = b * b - 4 * a * c;
	
	// 无实数解
	if (det < -1e-10) {
		return -1;
	}
	
	// 判别式接近0，视为重根
	if (fabs(det) < 1e-10) {
		det = 0.0;
	}

	// 计算两个根（此时 det >= 0）
	double sqrt_det = sqrt(det);
	double denominator = 2.0 * a;
	
	// 两个不同的实数根或重根
	ans[0] = (-b - sqrt_det) / denominator;
	ans[1] = (-b + sqrt_det) / denominator;
	
	// 从小到大排序
	if (ans[1] < ans[0]) {
		double tmp = ans[1];
		ans[1] = ans[0];
		ans[0] = tmp;
	}

	return 0;
}

// 解三次方程, 盛金公式
static double solve_cubic_equation(double *k, double *ans) {
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

	return -1.0;
}


/***********************************************************************
 *                        DoubleSCurve                                 *
 ***********************************************************************/
DoubleSCurve::DoubleSCurve() {
	this->clear();
}

void DoubleSCurve::clear() {
	m_vmax = 2, m_amax = 5, m_jmax = 5;

	m_sign = 1;
	m_scale = 1.0, m_offset = 0.0;
	m_reserveTime = 0.0;

	m_q0 = m_q1 = m_v0 = m_v1 = 0;
	m_Tj1 = m_Tj2 = m_Ta = m_Tv = m_Td = m_T = 0;

	m_alima = m_alimd = m_vlim = 0;
	m_s1 = m_s2 = m_s3 = m_s4 = m_s5 = m_s6 = 0;

	m_doneFlag = false;
	m_vp = 0.0;

	m_vmin = -m_vmax;
	m_amin = -m_amax;
	m_jmin = -m_jmax;
}

int DoubleSCurve::calc_plan_param() {

	// 速度、加速度上确界
	m_alima = m_jmax * m_Tj1;
	m_alimd = -m_jmax * m_Tj2;
	m_vlim = m_v0 + (m_Ta - m_Tj1)*m_alima;
	m_T = m_Ta + m_Tv + m_Td;

	// 不同阶段的运动位移
	double t = m_Tj1;
	m_s1 = m_v0 * t + m_jmax * t*t*t / 6;
	t = m_Ta - m_Tj1;
	m_s2 = m_v0 * t + m_alima / 6 * (3 * t*t - 3 * m_Tj1*t + m_Tj1 * m_Tj1);
	m_s3 = (m_vlim + m_v0) * m_Ta / 2;
	t = m_Tv;
	m_s4 = (m_vlim + m_v0) * m_Ta / 2 + m_vlim * t;
	t = m_Tj2;
	m_s5 = m_q1 - m_q0 - (m_vlim + m_v1)*m_Td / 2 + m_vlim * t - m_jmax * t*t*t / 6;
	t = m_Td - m_Tj2;
	m_s6 = m_q1 - m_q0 - (m_vlim + m_v1)*m_Td / 2 + m_vlim * t + m_alimd / 6 * (3 * t * t - 3 * m_Tj2 * t + m_Tj2 * m_Tj2);

	return 0;
}

int DoubleSCurve::set_condition(double begPos, double endPos, double begVel, double endVel) {
	m_q0 = begPos;
	m_q1 = endPos;
	m_v0 = begVel;
	m_v1 = endVel;

	m_sign = (m_q0 > m_q1) ? -1 : 1;
	m_q0 *= m_sign;
	m_q1 *= m_sign;
	m_v0 *= m_sign;
	m_v1 *= m_sign;
	m_vmax = (m_sign + 1) / 2 * m_vmax + (m_sign - 1) / 2 * m_vmin;
	m_vmin = (m_sign + 1) / 2 * m_vmin + (m_sign - 1) / 2 * m_vmax;
	m_amax = (m_sign + 1) / 2 * m_amax + (m_sign - 1) / 2 * m_amin;
	m_amin = (m_sign + 1) / 2 * m_amin + (m_sign - 1) / 2 * m_amax;
	m_jmax = (m_sign + 1) / 2 * m_jmax + (m_sign - 1) / 2 * m_jmin;
	m_jmin = (m_sign + 1) / 2 * m_jmin + (m_sign - 1) / 2 * m_jmax;

	m_xt[0] = begPos;
	m_xt[1] = m_xt[2] = m_xt[3] = 0;

	return 0;
}

int DoubleSCurve::set_constraint(double maxVel, double maxAcc, double maxJerk) {
	m_vmax = fabs(maxVel);
	m_amax = fabs(maxAcc);
	m_jmax = maxJerk > 0 ? fabs(maxJerk) : 10 * m_amax;

	m_vmin = -m_vmax;
	m_amin = -m_amax;
	m_jmin = -m_jmax;
	return 0;
}

int DoubleSCurve::set_reserve_time(double time) {
	m_reserveTime = time;
	return 0;
}

int DoubleSCurve::plan() {
	// 不考虑加速度限制，两段加速到达目标点时，加加速时间Tjs1
	double Tjs1 = std::sqrt(std::fabs(m_v1 - m_v0) / m_jmax);
	// 到达最大加速度时间，到达后需要匀加速阶段，即三段加速到达目标点，此时加加速时间Tjs2
	double Tjs2 = m_amax / m_jmax;
	double Tjs = Tjs1;
	
	// - 轨迹合法性检测: 单个加速/减速阶段即达到目标点速度，验证最短轨迹长度
	bool valid = true;
	double minDis = 0.0;
	// 两段加速的最短运动距离
	if (Tjs1 < Tjs2) {
		minDis = Tjs * (m_v0 + m_v1);
	}
	// 三段加速的最短运动距离
	else {
		Tjs = Tjs2;
		minDis = (m_v0 + m_v1) / 2 * (Tjs + std::fabs(m_v1 - m_v0) / m_jmax);
	}
	// 规划失败，无法在给定约束下通过双S曲线到达目标位置，速度规划时需要规避这种情况
	if (m_q1 - m_q0 < minDis) {
		valid = false;
		return 1;
	}

	// - 计算各阶段时间
	// Case 1: vlim = vmax
	// 加速阶段达到最大加速度
	if ((m_vmax-m_v0)*m_jmax < m_amax*m_amax) {
		m_Tj1 = std::sqrt(std::fabs(m_vmax - m_v0) / m_jmax);
		m_Ta = 2 * m_Tj1;
	}
	else {
		m_Tj1 = m_amax / m_jmax;
		m_Ta = m_Tj1 + (m_vmax - m_v0) / m_amax;
	}
	// 减速阶段达到最大加速度
	if ((m_vmax - m_v1)*m_jmax < m_amax*m_amax) {
		m_Tj2 = std::sqrt(std::fabs(m_vmax - m_v1) / m_jmax);
		m_Td = 2 * m_Tj2;
	}
	else {
		m_Tj2 = m_amax / m_jmax;
		m_Td = m_Tj2 + (m_vmax - m_v1) / m_amax;
	}
	m_Tv = (m_q1 - m_q0) / m_vmax - m_Ta / 2 * (1 + m_v0 / m_vmax) - m_Td / 2 * (1 + m_v1 / m_vmax);

	// Case 2: vlim < vmax
	if (m_Tv < 0) {
		m_Tv = m_Ta = m_Td = 0.0;
		valid = false;
		double low = 0.0, upp = 1.0;
		for (int i = 0; i < 10; ++i) {
			double mid = (low + upp) / 2.0;
			// 当前加速度限制可以规划，二分区间上移
			bool moveLow = true;
			// 迭代解
			double tj, tj1, tj2, ta, td, alim = mid * m_amax;

			// Case 2.1: alim = amax
			double det = std::pow(alim, 4.0) / m_jmax / m_jmax + 2 * (m_v0*m_v0 + m_v1 * m_v1) + alim * (4 * (m_q1 - m_q0) - 2 * alim / m_jmax * (m_v0 + m_v1));
			tj = tj1 = tj2 = alim / m_jmax;
			ta = (alim*alim / m_jmax - 2 * m_v0 + std::sqrt(det)) / (2 * alim);
			td = (alim*alim / m_jmax - 2 * m_v1 + std::sqrt(det)) / (2 * alim);

			// 仅需一个加速阶段，可以规划 (发生在v0 > v1)
			if (ta < 0) {
				ta = tj1 = 0.0;
				td = 2 * (m_q1 - m_q0) / (m_v1 + m_v0);
				tj2 = (m_jmax * (m_q1 - m_q0) - std::sqrt(m_jmax*(m_jmax*(m_q1 - m_q0)*(m_q1 - m_q0) + (m_v1 + m_v0)*(m_v1 + m_v0)*(m_v1 - m_v0)))) / (m_jmax*(m_v1 + m_v0));
			}
			// 仅需一个减速阶段，可以规划 (发生在v0 < v1)
			else if (td < 0) {
				td = tj2 = 0;
				ta = 2 * (m_q1 - m_q0) / (m_v1 + m_v0);
				tj1 = (m_jmax * (m_q1 - m_q0) - std::sqrt(m_jmax*(m_jmax*(m_q1 - m_q0)*(m_q1 - m_q0) - (m_v1 + m_v0)*(m_v1 + m_v0)*(m_v1 - m_v0)))) / (m_jmax*(m_v1 + m_v0));
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
				m_Ta = ta;
				m_Tj1 = tj1;
				m_Td = td;
				m_Tj2 = tj2;
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
	double h = m_q1 - m_q0;

	m_T = Tall;
	m_Ta = m_Td = Tacc;
	m_Tj1 = m_Tj2 = Tjerk;
	m_Tv = Tall - m_Ta - m_Td;
	
	double denom = (1 - alpha) * m_T;
	m_vmax = h / denom;
	denom *= alpha * (1 - beta) * m_T;
	m_amax = h / denom;
	denom *= alpha * beta * m_T;
	m_jmax = h / denom;

	m_vmin = -m_vmax;
	m_amin = -m_amax;
	m_jmin = -m_jmax;

	// - 计算最大速度、加速度
	calc_plan_param();

	return 0;
}

double DoubleSCurve::get_duration() {
	return m_T;
}

double DoubleSCurve::get_Ta() {
	return m_Ta;
}

double DoubleSCurve::get_Td() {
	return m_Td;
}

double DoubleSCurve::get_Tv() {
	return m_Tv;
}

double DoubleSCurve::get_vp() {
	return m_vp;
}

double DoubleSCurve::get_q1() {
	return m_q1;
}

bool DoubleSCurve::done() {
	return m_doneFlag;
}

double DoubleSCurve::get_offset() const {
	return m_offset;
}

double DoubleSCurve::get_scale() const {
	return m_scale;
}

double DoubleSCurve::get_pos(double t) {
	
	// 时间缩放: 
	t = (t - m_offset) / m_scale;

	// 插补完成标志
	if (t + m_reserveTime > m_T) {
		t = m_T - m_reserveTime;
		m_doneFlag = true;
	}
	else {
		m_doneFlag = false;
	}

	double q = m_q0, v = m_v0, a = 0.0, j = 0.0;
	if (t < 0) {
		q = m_q0;
		v = m_v0;
	}
	// 加速阶段
	else if (t < m_Tj1) {
		q = m_q0 + m_v0 * t + m_jmax * t*t*t / 6;
		v = m_v0 + m_jmax * t*t / 2;
		a = m_jmax * t;
		j = m_jmax;
	}
	else if (t < m_Ta - m_Tj1) {
		q = m_q0 + m_v0 * t + m_alima / 6 * (3 * t*t - 3 * m_Tj1*t + m_Tj1 * m_Tj1);
		v = m_v0 + m_alima * (t - m_Tj1 / 2);
		a = m_alima;
		j = 0;
	}
	else if (t < m_Ta) {
		double curT = m_Ta - t;
		q = m_q0 + (m_vlim + m_v0) * m_Ta / 2 - m_vlim * curT - m_jmin * curT*curT*curT / 6;
		v = m_vlim + m_jmin * curT*curT / 2;
		a = -m_jmin * curT;
		j = m_jmin;
	}
	// 匀速阶段
	else if (t < m_Ta + m_Tv) {
		q = m_q0 + (m_vlim + m_v0) * m_Ta / 2 + m_vlim * (t - m_Ta);
		v = m_vlim;
		a = 0;
		j = 0;
	}
	// 减速阶段
	else if (t < m_T - m_Td + m_Tj2) {
		double curT = t - m_T + m_Td;
		q = m_q1 - (m_vlim + m_v1)*m_Td / 2 + m_vlim * curT - m_jmax * curT*curT*curT / 6;
		v = m_vlim - m_jmax * curT*curT / 2;
		a = -m_jmax * curT;
		j = m_jmin;
	}
	else if (t < m_T - m_Tj2) {
		double curT = t - m_T + m_Td;
		q = m_q1 - (m_vlim + m_v1)*m_Td / 2 + m_vlim * curT + m_alimd / 6 * (3 * curT*curT - 3 * m_Tj2*curT + m_Tj2 * m_Tj2);
		v = m_vlim + m_alimd * (curT - m_Tj2 / 2);
		a = m_alimd;
		j = 0;
	}
	else if (t < m_T) {
		double curT = m_T - t;
		q = m_q1 - m_v1 * curT - m_jmax * curT*curT*curT / 6;
		v = m_v1 + m_jmax * curT*curT / 2;
		a = -m_jmax * curT;
		j = m_jmax;
	}
	// 超出规划时间
	else {
		q = m_q1;
		v = m_v1;
	}

	q *= m_sign;
	v *= m_sign;
	a *= m_sign;
	j *= m_sign;

	m_vp = v;

	return q;
}

int DoubleSCurve::get_onlineState() {
	return m_onlineState;
}

int DoubleSCurve::set_onlineState(int state) {
	if (state == m_onlineState)
		return 1;
	
	m_onlineStateSignal = state;
	return 0;
}

int DoubleSCurve::switch_online_state() {
	// --- 1. 不需要切换状态
	// 1.1 方向相同
	if (m_onlineStateSignal * m_onlineState > 0) {
		return 0;
	}
	// 1.2 停止信号，当前正在停止或已经停止
	else if (m_onlineStateSignal == 0 && std::abs(m_onlineState) != 1) {
		return 0;
	}

	// --- 2. 切换状态: 后续可以改为从当前位置规划到限位，目前先用来测试停止规划
	int state = m_onlineStateSignal;

	// 停止，不改变曲线方向
	if (std::abs(m_onlineState) == 1 && state == 0) {
		// 切换到停止状态
		m_onlineState = m_onlineState > 0 ? 2 : -2;
		// 速度方向与实际点动方向不一致，规划方向需要反转
		if (m_sign * m_xt[1] * m_onlineState < 0) {
			m_onlineState = (m_sign * m_xt[1] < 0) ? -2 : 2;
			for (int i = 0; i < 4; ++i)
				m_xt[i] *= -1;
		}
		m_sign = m_onlineState < 0 ? -1 : 1;

		// 将当前状态保存为减速规划的起点状态
		m_q0 = m_xt[0];
		m_v0 = m_xt[1];
		m_a0 = m_xt[2];
		// 规划减速段
		double Tdi[3];
		m_q1 = m_sign * plan_decccel_online_interp(Tdi);
		// 设定减速段时长
		apply_deccel_plan(Tdi);
	}
	// 静止切换到运动 / 运动过程中切换状态
	else if (state != 0) {
		// 方向改变: 零变负，负变正
		if ((m_onlineState == 0 && state < 0) || (m_onlineState * state < 0)) {
			for (int i = 0; i < 4; ++i)
				m_xt[i] *= -1;
		}
		m_onlineState = state;
		m_sign = m_onlineState < 0 ? -1 : 1;
	}

	return 0;
}

int DoubleSCurve::online_interp(double dt) {
	// 检测状态切换
	switch_online_state();

	// 当前为停止状态，无需插补
	if (m_onlineState == 0)
		return 0;

	double x = m_xt[0], v = m_xt[1], a = m_xt[2], j = 0;
	// 前进
	if (std::abs(m_onlineState) == 1) {
		m_decCnt = 0;
		double vc = m_xt[1] - m_xt[2] * m_xt[2] / (2 * m_jmin);
		double ac = m_xt[2];

		// 加加速阶段
		if (vc < m_vmax && ac < m_amax) {
			j = m_jmax;
		}
		// 匀加速阶段
		else if (vc < m_vmax && ac >= m_amax) {
			j = 0.0;
		}
		// 减加速阶段
		else if (vc >= m_vmax && ac > 0) {
			j = m_jmin;
		}
		// 匀速阶段
		else if (vc >= m_vmax && ac <= 0) {
			j = 0.0;
		}

		// 更新插补状态
		a = m_xt[2] + dt * (j + m_xt[3]) / 2;
		v = m_xt[1] + dt * (a + m_xt[2]) / 2;
		x = m_xt[0] + dt * (v + m_xt[1]) / 2;
	}
	// 停止
	else if (std::abs(m_onlineState) == 2) {
		m_decCnt++;
		double decT = m_decCnt * dt;
		// 按减速段规划运动至停止
		if (decT < m_Tj2a) {
			j = m_jmin;
			a = m_a0 + m_jmin * decT;
			v = m_v0 + m_a0 * decT + m_jmin * decT * decT / 2;
			x = m_q0 + m_v0 * decT + m_a0 * decT * decT / 2 + m_jmin * decT * decT * decT / 6;
		}
		else if (decT < m_Td - m_Tj2b) {
			j = 0;
			a = m_alimd;
			v = m_v0 + m_alimd * (decT - m_Tj2a / 2);
			x = m_q0 + m_v0 * decT + m_a0 * m_Tj2a * m_Tj2a / 2 + m_alimd * (3 * decT * decT + 3 * decT * m_Tj2a + m_Tj2a * m_Tj2a);
		}
		else if (decT < m_Td) {
			j = m_jmax;
			a = -m_jmax * (m_Td - decT);
			v = m_jmax * (m_Td - decT) * (m_Td - decT) / 2;
			x = m_q1 - m_jmax * (m_Td - decT) * (m_Td - decT) * (m_Td - decT) / 6;
		}
		else {
			// 结束后方向切回正向，查询时会反转，这里提前反转
			x *= m_sign;
			v *= m_sign;
			a *= m_sign;

			// 运动结束后状态清零
			m_onlineState = 0;
			// 切换为正向插补
			for (int i = 0; i < 4; ++i)
				m_xt[i] *= m_sign;
			m_sign = 1;
		}
	}

	// 输出到正确方向
	m_xt[0] = x;
	m_xt[1] = v;
	m_xt[2] = a;
	m_xt[3] = j;

	return m_onlineState;
}

int DoubleSCurve::get_cur_state(double state[4]) {
	for (int i = 0; i < 4; ++i) {
		state[i] = m_xt[i] * m_sign;
	}
	return m_onlineState;
}

double DoubleSCurve::plan_decccel_online_interp(double Tdi[3]) {
	// 能达到匀减速度阶段 (3.36)
	double Tj2a = (m_amin - m_xt[2]) / m_jmin;
	double Tj2b = (0 - m_amin) / m_jmax;
	double Td = -m_xt[1] / m_amin + Tj2a * (m_amin - m_xt[2]) / (2 * m_amin) + Tj2b * m_amin / (2 * m_amin);

	// 不能达到匀加速度阶段 (3.37)
	if (Td < Tj2a + Tj2b) {
		Tj2a = -m_xt[2] / m_jmin + sqrt((m_jmax - m_jmin)*(m_xt[2] * m_xt[2] * m_jmax - m_jmin * 2 * m_jmax*m_xt[1])) / (m_jmin*(m_jmin - m_jmax));
		Tj2b = sqrt((m_jmax - m_jmin)*(m_xt[2] * m_xt[2] * m_jmax - m_jmin * 2 * m_jmax*m_xt[1])) / (m_jmax*(m_jmax - m_jmin));
		Td = Tj2a + Tj2b;
	}
	// 计算终点位置
	double hk = m_xt[2] * Td * Td / 2 + (m_jmin * Tj2a * (3 * Td * Td - 3 * Td * Tj2a + Tj2a * Tj2a) + m_jmax * Tj2b * Tj2b * Tj2b) / 6 + Td * m_xt[1];

	// 如果在 Td 阶段重新规划减速时，Tj2a 可能会是负值
	if (Tj2a < 0) {
		Tj2a = 0.0;
		Tj2b = 2 * std::fabs(m_xt[1] / m_xt[2]);
		Td = Tj2a + Tj2b;
		m_jmax = std::abs(m_xt[2]) / Td;
		if (Tj2b < 0) {
			printf("Error at dec_plan: %d, %f, %f, %f, %f -> T: %f, %f\n", m_sign, m_xt[0], m_xt[1], m_xt[2], m_xt[3], Tj2a, Tj2b);
		}
		hk = m_jmax * Td * Td * Td / 6;
	}
	// 如果速度超调 v < 0 时仍按正向减速规划, Tj2b 可能会是负值
	else if (Tj2b < 0) {
		Tj2b = 0;
		Tj2a = 2 * m_xt[1] / m_xt[2];
		Td = Tj2a + Tj2b;
		m_jmax = m_xt[2] / Td;
		hk = m_jmax * Td * Td * Td / 6;
		if (Tj2a < 0) {
			printf("Error at dec_plan: %d, %f, %f, %f, %f -> T: %f, %f\n", m_sign, m_xt[0], m_xt[1], m_xt[2], m_xt[3], Tj2a, Tj2b);
		}
	}

	Tdi[0] = Tj2a;
	Tdi[1] = Tj2b;
	Tdi[2] = Td;


	return m_sign * (m_xt[0] + hk);
}

int DoubleSCurve::apply_deccel_plan(double Tdi[3]) {
	m_Tj2a = Tdi[0];
	m_Tj2b = Tdi[1];
	m_Td  = Tdi[2];

	m_alimd = m_a0 + m_jmin * m_Tj2a;

	return 0;
}

int DoubleSCurve::displacement(double dt, double k) {
	m_offset = dt;
	m_scale = k;
	return 0;
}

double DoubleSCurve::get_remain_dist(double dt) {
	int num = m_T / dt;
	double endT = num * dt;
	return m_q1 - get_pos(endT);
}

double DoubleSCurve::get_remain_time(double dt) {
	int num = m_T / dt;
	double endT = num * dt;
	return m_T - endT;
}

double DoubleSCurve::get_max_speed(double ds) {

	double Tjk, Tacc;
	// 加速到最大速度时未达到最大加速度 (3.19)
	if ((m_vmax - m_v0)*m_jmax < m_amax*m_amax) {
		Tjk = std::sqrt(std::fabs(m_vmax - m_v0) / m_jmax);
		Tacc = 2 * Tjk;
	}
	else {
		Tjk = m_amax / m_jmax;
		Tacc = Tjk + (m_vmax - m_v0) / m_amax;
	}

	// 1. 加速距离内能达到最大速度 (vlim = vmax)
	if (ds > Tacc * (m_v0 + m_vmax) / 2) {
		return m_vmax;
	}

	// 2. 加速距离内不能达到最大速度 (vlim < vmax)，根据是否达到最大加速度区分加速阶段的段数
	double sol[3] = { 0.0 };
	int solNum = 3;
	// 两段加速刚好达到最大加速度: Tj = a/J; ds = (v0+v1)*Tj = (2*v0*J + a*a)*a/(J*J)
	if (ds > (2 * m_v0 * m_jmax + m_amax * m_amax) * m_amax / (m_jmax * m_jmax)) {
		// 2.1. 需要三段加速到达目标距离 (alim = amax)(3-27): ds = Ta*(v0+v1)/2; v1-v0 = a*(Ta-Tj)
		Tjk = m_amax / m_jmax;
		double coeff[3] = { 0 };
		coeff[0] = m_amax * m_amax / m_jmax * m_v0 - 2 * m_amax * ds - m_v0 * m_v0;
		coeff[1] = m_amax * m_amax / m_jmax;
		coeff[2] = 1;
		solve_quadratic_equation(coeff, sol);
	}
	else {
		// 2.2. 两段加速到达目标距离 (alim < amax)
		double coeff[4] = { -m_v0 * m_v0 * m_v0 - ds * ds * m_jmax, -m_v0 * m_v0, m_v0, 1.0 };
		solNum = 2;
		solve_cubic_equation(coeff, sol);
	}

	// 最小正解
	double ans = 0.0;
	for (int i = 0; i < solNum; ++i) {
		if (sol[i] > 0) {
			ans = sol[i];
			break;
		}
	}

	// 速度冗余，防止计算误差导致规划时最大速度超出限制
	double margin = 1e-9;
	return (ans < margin) ? 0 : ans - margin;
}

double DoubleSCurve::calc_time_PiTPe(double ds) {
	// 当前运动距离
	double s = m_q1 - m_q0 - ds;

	double sol[3] = { 0.0 }, t0 = 0.0;
	double solNum = 2, solShift = 0.0;
	if (s < 0) {
		sol[0] = m_T;
		solNum = 1;
	}
	else if (s < m_s1) {
		double coeff[4] = { -s, m_v0, 0, m_jmax / 6 };
		solve_cubic_equation(coeff, sol);
		solNum = 3;
	}
	else if (s < m_s2) {
		double coeff[3] = { m_alima*m_Tj1*m_Tj1 / 6 - s, m_v0 - m_alima * m_Tj1 / 2, m_alima / 2 };
		solve_quadratic_equation(coeff, sol);
	}
	else if (s < m_s3) {
		double coeff[4] = { s - (m_vlim + m_v0)*m_Ta / 2, m_vlim, 0, m_jmin / 6 };
		solve_cubic_equation(coeff, sol);
		solNum = 3;
		solShift = -m_Ta;
	}
	else if (s < m_s4) {
		sol[0] = (s - (m_vlim + m_v0)*m_Ta / 2) / m_vlim + m_Ta;
		solNum = 1;
	}
	else if (s < m_s5) {
		double coeff[4] = { s - (m_vlim + m_v1)*m_Td / 2, -m_vlim, 0, m_jmax / 6 };
		solve_cubic_equation(coeff, sol);
		solNum = 3;
		solShift = m_T - m_Td;
	}
	else if (s < m_s6) {
		double coeff[3] = { m_alimd*m_Tj2*m_Tj2 / 6 - (m_vlim + m_v1)*m_Tj2 / 2 - s, m_vlim - m_alimd * m_Tj2 / 2, m_alimd / 2 };
		solve_quadratic_equation(coeff, sol);
		solShift = m_T - m_Td;
	}
	else if (s < m_q1 - m_q0) {
		double coeff[4] = { -s, m_v1, 0, m_jmax / 6 };
		solve_cubic_equation(coeff, sol);
		solNum = 3;
		solShift = -m_T;
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
	ans = m_T - std::fabs(ans + solShift);

	return ans;
}


/***********************************************************************
 *                        B E Z I E R                                  *
 ***********************************************************************/
 // 贝塞尔曲线位置
int bezier_positioin(int m, const double ctr[][3], double u, double ans[3]) {
	double *Q = (double *)malloc(sizeof(double) * (m + 1));

	// 第J维度
	for (int j = 0; j < 3; ++j) {
		for (int i = 0; i <= m; ++i) {
			Q[i] = ctr[i][j];
		}

		// DeCasteljau算法降维 m 次, 相比直接计算效率稍低，但是舍入误差更小
		for (int k = 1; k <= m; ++k) {
			for (int i = 0; i <= m - k; ++i) {
				Q[i] = (1.0 - u)*Q[i] + u * Q[i + 1];
			}
		}

		ans[j] = Q[0];
	}

	// 释放内存
	free(Q);
	return 0;
}

// 贝塞尔曲线导数
double bezier_derivatives(int m, const double ctr[][3], double u, double ans[3]) {
	// 贝塞尔曲线的导数是低阶的贝塞尔曲线
	double(*pnt)[3] = (double(*)[3])malloc(sizeof(double) * 3 * m);

	// 导数曲线控制点
	for (int i = 0; i < m; ++i) {
		for (int k = 0; k < 3; ++k) {
			pnt[i][k] = ctr[i + 1][k] - ctr[i][k];
		}
	}

	// b'(m, u) = m * b(m-1, u)
	bezier_positioin(m - 1, pnt, u, ans);
	for (int i = 0; i < 3; ++i) {
		ans[i] *= m;
	}

	// 释放内存
	free(pnt);
	return sqrt(ans[0] * ans[0] + ans[1] * ans[1] + ans[2] * ans[2]);
}

// 贝塞尔曲线段长度 - 辛普森积分
double bezier_dist(int m, const double ctr[][3], double a, double b, int n) {
	// 区间间隔，包含一个奇数节点和偶数节点
	double h = (b - a) / n;
	double deriv[3];

	// 奇数节点导数
	double so = bezier_derivatives(m, ctr, a + h * 0.5, deriv);
	// 偶数节点导数，除去0和2n
	double se = 0.0;
	// 闭区间等分成2n个小区间
	for (int i = 1; i < n; ++i) {
		so += bezier_derivatives(m, ctr, a + h * (i + 0.5), deriv);
		se += bezier_derivatives(m, ctr, a + h * i, deriv);
	}

	// ans = h/6 * (f(a) + 4*so + 2*se + f(b))
	double ans = bezier_derivatives(m, ctr, a, deriv) + bezier_derivatives(m, ctr, b, deriv);
	ans += 4 * so + 2 * se;
	ans *= h / 6;

	return ans;
}

// 贝塞尔曲线插补
double bezier_interp(int m, const double ctr[][3], double curU, double detS, int num) {
	// --- 1. 四阶龙格库塔: y(s) = u(s), 求u(s+ds)
	double deriv[3] = { 0.0 };
	double predU = curU;
	// 起点斜率
	double k1 = 1.0 / bezier_derivatives(m, ctr, curU, deriv);

	// 中间点斜率，使用u估计值处的导数近似半步处的导数
	predU = curU + k1 * detS / 2;
	double k2 = 1.0 / bezier_derivatives(m, ctr, predU, deriv);

	// 修正的中间点斜率
	predU = curU + k2 * detS / 2;
	double k3 = 1.0 / bezier_derivatives(m, ctr, predU, deriv);

	// 终点斜率
	predU = curU + k3 * detS;
	double k4 = 1.0 / bezier_derivatives(m, ctr, predU, deriv);

	// 终点函数值
	double nextU = curU + detS / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
	return nextU;

	// --- 2. 二分法
	double beg = curU, end = 1.0;
	int maxIteNum = 20;

	// 最大迭代次数
	double lastU = 1.0;
	for (int i = 0; i < maxIteNum; ++i) {
		// 中点参数
		double U = (beg + end) / 2;

		double dis = bezier_dist(m, ctr, curU, U, num);

		// 更新区间端点
		if (dis > detS)
			end = U;
		else
			beg = U;
	}

	return (beg + end) / 2;
}

