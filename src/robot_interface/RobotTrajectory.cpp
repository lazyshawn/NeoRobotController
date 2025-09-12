
#include "robot_interface/RobotTrajectory.h"

//namespace RobotTrajectory {

Eigen::Matrix<DT_scale, 3, 1> get_equivalent_zyx_euler(const Eigen::Matrix<DT_scale, 3, 1>& endEuler) {

	Eigen::Matrix<DT_scale, 3, 1> equivEuler = endEuler;

	while (std::fabs(equivEuler[1]) - 180 > 0) {
		equivEuler[1] += equivEuler[1] > 0 ? -360 : 360;
	}

	// Ry == 90
	if (endEuler[1] == 90) {
		equivEuler[0] = 0;
		equivEuler[2] = endEuler[2] - endEuler[0];
	}
	// Ry == -90
	else if (endEuler[1] == -90) {
		equivEuler[0] = 0;
		equivEuler[2] = endEuler[2] + endEuler[0];
	}
	else {
		equivEuler[0] = endEuler[0] + 180;
		equivEuler[1] = 180 - endEuler[1];
		equivEuler[2] = endEuler[2] + 180;
	}
	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(equivEuler[i]) - 180 > 0) {
			equivEuler[i] += equivEuler[i] > 0 ? -360 : 360;
		}
	}

	return equivEuler;
}

Eigen::Matrix<DT_scale, 3, 1> get_zyx_euler_distance(Eigen::Matrix<DT_scale, 3, 1>& begEuler, Eigen::Matrix<DT_scale, 3, 1>& endEuler, bool chooseMimumDist) {
	Eigen::Matrix<DT_scale, 3, 1> ans(0, 0, 0);

	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(begEuler[i]) - 180 > 0) {
			begEuler[i] += begEuler[i] > 0 ? -360 : 360;
		}
		while (std::fabs(endEuler[i]) - 180 > 0) {
			endEuler[i] += endEuler[i] > 0 ? -360 : 360;
		}
	}

	// equivalent of endEuler
	Eigen::Matrix<DT_scale, 3, 1> equivEuler = get_equivalent_zyx_euler(endEuler);
	if (endEuler[1] == 90) {
		equivEuler[0] = (equivEuler[2] > 0) ? (std::min)(begEuler[0], begEuler[2]) : (std::max)(begEuler[0], begEuler[2]);
		equivEuler[2] += equivEuler[0];
	}
	else if (endEuler[1] == -90) {
		DT_scale sumQ = endEuler[2] + endEuler[0];
		equivEuler[0] = begEuler[0];
		equivEuler[2] -= equivEuler[0];
	}
	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(equivEuler[i]) - 180 > 0) {
			equivEuler[i] += equivEuler[i] > 0 ? -360 : 360;
		}
	}

	// 欧拉角相对值
	Eigen::Matrix<DT_scale, 3, 1> endRel(0, 0, 0), equivRel(0, 0, 0);
	DT_scale endSum = 0.0, equivSum = 0.0;
	for (size_t i = 0; i < 3; ++i) {
		DT_scale directDist = endEuler[i] - begEuler[i];
		DT_scale hopDist = 360 - std::fabs(endEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			endRel[i] = directDist;
		}
		else {
			// 此时 begEuler[i] ！= 0 成立
			endRel[i] = begEuler[i] < 0 ? -hopDist : hopDist;
		}
		endSum += std::fabs(endRel[i]);

		directDist = equivEuler[i] - begEuler[i];
		hopDist = 360 - std::fabs(equivEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			equivRel[i] = directDist;
		}
		else {
			equivRel[i] = begEuler[i] < 0 ? -hopDist : hopDist;
		}
		equivSum += std::fabs(equivRel[i]);
	}

	// 选择最短 / 最长的相对运动距离
	if (chooseMimumDist) {
		//ans = endSum < equivSum ? endRel : equivRel;
		if (endSum < equivSum) {
			ans = endRel;
		}
		else {
			ans = equivRel;
			endEuler = equivEuler;
		}
	}
	else {
		//ans = endSum < equivSum ? equivRel : endRel;
		if (endSum > equivSum) {
			ans = endRel;
		}
		else {
			ans = equivRel;
			endEuler = equivEuler;
		}
	}

	return ans;
}

Eigen::Matrix<DT_scale, 3, 1> get_zyx_euler_distance(Eigen::Matrix<DT_scale, 3, 1>& begEuler, Eigen::Matrix<DT_scale, 3, 1>& midEuler, Eigen::Matrix<DT_scale, 3, 1>& endEuler) {
	// 欧拉角转换到相对运动: beg -> mid -> end
	Eigen::Matrix<DT_scale, 3, 1> relEuler = get_zyx_euler_distance(begEuler, midEuler);
	relEuler += get_zyx_euler_distance(midEuler, endEuler);

	return relEuler;
}

std::vector<DT_scale> calc_traj_info(const std::vector<DT_scale>& begPnt, const std::vector<DT_scale>& midPnt, const std::vector<DT_scale>& endPnt, int mode) {

	std::vector<DT_scale> ans(7,0);

	Eigen::Matrix<DT_scale, 3, 1> begPos(begPnt[0], begPnt[1], begPnt[2]), endPos(endPnt[0], endPnt[1], endPnt[2]), midPos(midPnt[0], midPnt[1], midPnt[2]);
	std::vector<DT_scale> knot(3, 0), dir(3, 0);
	DT_scale dist = 0.0;
	bool isLine = (mode == 0);

	if (mode == 1) {
		Eigen::Matrix<DT_scale, 3, 1> a = begPos - midPos, b = endPos - midPos;
		// 当直线处理
		//if (a.cross(b).squaredNorm() < 1e-9) {
		//	info.head(3) = (endPos - begPos).normalized();
		//	info[3] = (endPos - begPos).norm();
		//	isLine = true;
		//}

		// 圆心位置
		Eigen::Matrix<DT_scale, 3, 1> cent = (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + midPos;
		//Eigen::Matrix<DT_scale, 3, 1> cent;
		//make_circle({ begPos, midPos, endPos }, cent);
		// 半径方向
		Eigen::Matrix<DT_scale, 3, 1> op1 = (begPos - cent).normalized(), op2 = (midPos - cent).normalized(), op3 = (endPos - cent).normalized();
		// 法线方向
		Eigen::Matrix<DT_scale, 3, 1> n12 = op1.cross(op2), n13 = op1.cross(op3);
		// 圆弧运动平面的法线方向
		Eigen::Matrix<DT_scale, 3, 1> normal = op1.cross(op3);
		// 半径夹角
		DT_scale q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
		// 圆心角度
		DT_scale theta = std::acos(op1.dot(op3));

		// 修正圆心角和法向量
		// 2,3 在 1 的两侧
		if (n12.dot(n13) < 0) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}
		// q12 > q13
		else if (q12 > q13) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}
		else if (q13 > q12) {
			normal = n12;
		}

		normal.normalize();
		normal *= theta;
		knot.assign(cent.data(), cent.data() + 3);
		dist = (begPos - cent).norm() * theta;
		dir.assign(normal.data(), normal.data() + 3);
	}

	if (isLine) {
		Eigen::Matrix<DT_scale, 3, 1> lineDir = (endPos - begPos).normalized();

		// 计算轨迹信息
		knot.assign(begPos.data(), begPos.data() + 3);
		dist = (endPos - begPos).norm();
		dir.assign(lineDir.data(), lineDir.data() + 3);
	}

	for (size_t i = 0; i < 3; ++i) {
		ans[i] = knot[i];
		ans[i + 4] = dir[i];
	}
	ans[3] = dist;

	return ans;
}


//TrajectoryPoint partition_trajectory(const TrajectoryPoint& preTraj, const TrajectoryPoint& curTraj, DT_scale begRatio, DT_scale endRatio, int mode) {
//
//	// 获取节点目标位置
//	auto curPoint = curTraj.mainPoint;
//	auto prePoint = preTraj.mainPoint;
//	auto midPoint = curTraj.auxPoint;
//
//	int num = curPoint.size();
//	// 分段结果
//	TrajectoryPoint ans(num);
//	ans.trajType = curTraj.trajType;
//	std::vector<DT_scale> relEndMove(num, 0);
//
//	// 附加轴相对变化量
//	for (size_t i = 0; i < num; ++i)
//		relEndMove[i] = curPoint[i] - prePoint[i];
//
//	// 欧拉角相对变化量
//	if (curPoint.size() > 5) {
//		auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(prePoint[3], prePoint[4], prePoint[5]);
//		auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(midPoint[3], midPoint[4], midPoint[5]);
//		auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(curPoint[3], curPoint[4], curPoint[5]);
//		auto relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
//		for (size_t i = 0; i < 3; ++i) {
//			relEndMove[3 + i] = relEuler[i];
//		}
//	}
//
//	bool isArc = (curTraj.trajType == TrajType::Arc);
//
//	// 计算位置分量
//	auto trajInfo = calc_traj_info(prePoint, midPoint, curPoint, isArc);
//	DT_scale partial = 0;
//	// 圆弧运动
//	if (isArc) {
//		// Eigen 类型的点位，用于计算
//		Eigen::Vector3f rotNorm(trajInfo[4], trajInfo[5], trajInfo[6]), centerPos(trajInfo[0], trajInfo[1], trajInfo[2]);
//
//		// 轨迹总旋转角度
//		DT_scale theta = rotNorm.norm();
//		rotNorm.normalize();
//		// 起点处的半径
//		Eigen::Vector3f radiusDir(0, 0, 0);
//		for (size_t i = 0; i < 3; ++i) {
//			radiusDir[i] = prePoint[i] - centerPos[i];
//		}
//		// 分段点位置
//		Eigen::Vector3f arcPos;
//
//		// 中间点处的比例
//		partial = (mode == 0) ? (begRatio + endRatio) / 2 : (begRatio + endRatio) / 2 / (theta * radiusDir.norm());
//		arcPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
//		// 位置分量单独计算，姿态和附加值按线性累加
//		for (size_t i = 0; i < num; ++i) {
//			ans.auxPoint[i] = i < 3 ? arcPos[i] : (prePoint[i] + relEndMove[i] * partial);
//		}
//
//		// 终点处的比例
//		partial = (mode == 0) ? endRatio : endRatio / (theta * radiusDir.norm());
//		arcPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
//		// 位置分量单独计算，姿态和附加值按线性累加
//		for (size_t i = 0; i < num; ++i) {
//			ans.mainPoint[i] = i < 3 ? arcPos[i] : (prePoint[i] + relEndMove[i] * partial);
//		}
//	}
//	// 直线运动
//	else {
//		// 比例
//		partial = (mode == 0) ? (begRatio + endRatio) / 2 : (begRatio + endRatio) / 2 / trajInfo[3];
//		//if (partial > 1)
//		//	partial = 1;
//		for (size_t i = 0; i < num; ++i)
//			ans.auxPoint[i] = prePoint[i] + relEndMove[i] * partial;
//
//		// 比例
//		partial = (mode == 0) ? endRatio : endRatio / trajInfo[3];
//		//if (partial > 1)
//		//	partial = 1;
//		for (size_t i = 0; i < num; ++i)
//			ans.mainPoint[i] = prePoint[i] + relEndMove[i] * partial;
//	}
//
//	//if (std::fabs(endRatio-1) < 1e-2) {
//	//	ans.mainPoint = curPoint;
//	//}
//
//	return ans;
//}

std::vector<DT_scale> get_relative_distance(const std::vector<DT_scale>& beg, const std::vector<DT_scale>& mid, const std::vector<DT_scale>& end) {

	int num = end.size();
	std::vector<DT_scale> relEndMove(num, 0);
	// 终点到起点的相对运动
	for (int i = 0; i < num; ++i) {
		relEndMove[i] = end[i] - beg[i];
	}

	// 欧拉角转换到相对运动: beg -> mid -> end
	auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(beg[3], beg[4], beg[5]);
	auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(mid[3], mid[4], mid[5]);
	auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(end[3], end[4], end[5]);
	// 欧拉角相对值
	Eigen::Matrix<DT_scale, 3, 1> relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		relEndMove[3 + i] = relEuler[i];
	}

	return relEndMove;
}

/* *************************************** TrajectoryPoint *************************************** */
TrajectoryPoint::TrajectoryPoint() {
}

TrajectoryPoint::TrajectoryPoint(int num) {
	mainPoint = std::vector<DT_scale>(num, 0);
	auxPoint = std::vector<DT_scale>(num, 0);
}

int TrajectoryPoint::isJoint() const {

	if (trajType == TrajType::Joint) {
		return 1;
	}

	return 0;
}

int TrajectoryPoint::isLine() const {

	if (trajType == TrajType::Line || trajType == TrajType::Line_R) {
		return 1;
	}

	return 0;
}

int TrajectoryPoint::isArc() const {

	if (trajType == TrajType::Arc || trajType == TrajType::Arc_R) {
		return 1;
	}

	return 0;
}

int TrajectoryPoint::isCartesian() const {
	if (isLine() || isArc()) {
		return 1;
	}
	return 0;
}

int TrajectoryPoint::isBaseMotion() const {
	if (trajType == TrajType::Line_R || trajType == TrajType::Arc_R) {
		return 1;
	}
	return 0;
}

/* *************************************** SingleTrajectory *************************************** */
SingleTrajectory::SingleTrajectory() {

	trajType = TrajType::None;

}

SingleTrajectory::SingleTrajectory(const TrajectoryPoint& pnt, const TrajectoryConfig cfg) {

	set_point(pnt);
	set_config(cfg);

}

void SingleTrajectory::set_point(const TrajectoryPoint& newPoint) {

	trajType = newPoint.trajType;
	mainPoint = newPoint.mainPoint;
	auxPoint = newPoint.auxPoint;

}

TrajectoryPoint SingleTrajectory::get_point() const {
	TrajectoryPoint point;
	point.mainPoint = mainPoint;
	point.auxPoint = auxPoint;
	point.trajType = trajType;
	return point;
}

void SingleTrajectory::set_config(const TrajectoryConfig& cfg) {

	speed = cfg.speed;
	smooth = cfg.smooth;
	moveInBase = cfg.moveInBase;
	axisMask = cfg.axisMask;

	appendix = cfg.appendix;
}
TrajectoryConfig SingleTrajectory::get_config() {
	TrajectoryConfig cfg;

	cfg.speed = speed;
	cfg.smooth = smooth;
	cfg.moveInBase = moveInBase;
	cfg.axisMask = axisMask;

	cfg.appendix = appendix;

	return cfg;
}




/* *************************************** DiscreteTrajectory *************************************** */
int DiscreteTrajectory::calc_traj_info() {

	auto curTraj = get_curTraj();
	if (!curTraj.isCartesian() || !preTraj.isCartesian()) {
		return -1;
	}

	auto beg = preTraj.mainPoint;
	auto end = curTraj.mainPoint;
	auto mid = curTraj.auxPoint;

	Eigen::Matrix<DT_scale, 3, 1> begPos(beg[0], beg[1], beg[2]), endPos(end[0], end[1], end[2]), midPos(mid[0], mid[1], mid[2]);

	if (curTraj.isLine()) {
		Eigen::Matrix<DT_scale, 3, 1> lineDir = (endPos - begPos).normalized();

		// 计算轨迹信息
		this->knot.assign(begPos.data(), begPos.data() + 3);
		this->dist = (endPos - begPos).norm();
		this->dir.assign(lineDir.data(), lineDir.data() + 3);
	}
	else if (curTraj.isArc()) {
		//Eigen::Matrix<DT_scale, 7, 1> info;

		Eigen::Matrix<DT_scale, 3, 1> a = begPos - midPos, b = endPos - midPos;
		// 当直线处理
		//if (a.cross(b).squaredNorm() < 1e-9) {
		//	info.head(3) = (endPos - begPos).normalized();
		//	info[3] = (endPos - begPos).norm();
		//	return -1;
		//}

		// 圆心位置
		Eigen::Matrix<DT_scale, 3, 1> cent = (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + midPos;
		//Eigen::Matrix<DT_scale, 3, 1> cent;
		//make_circle({ begPos, midPos, endPos }, cent);
		// 半径方向
		Eigen::Matrix<DT_scale, 3, 1> op1 = (begPos - cent).normalized(), op2 = (midPos - cent).normalized(), op3 = (endPos - cent).normalized();
		// 法线方向
		Eigen::Matrix<DT_scale, 3, 1> n12 = op1.cross(op2), n13 = op1.cross(op3);
		n12.normalize();
		n13.normalize();
		// 圆弧运动平面的法线方向
		//Eigen::Matrix<DT_scale, 3, 1> normal = op1.cross(op3);
		Eigen::Matrix<DT_scale, 3, 1> normal = n13;
		// 半径夹角
		DT_scale q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
		// 圆心角度
		DT_scale theta = std::acos(op1.dot(op3));

		// 修正圆心角和法向量
		// 2,3 在 1 的两侧
		if (n12.dot(n13) < 0) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}
		// q12 > q13
		else if (q12 > q13) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}

		normal.normalize();
		//Eigen::Matrix<DT_scale, 3, 1> arcPos = Eigen::AngleAxisf(theta, normal) * (begPos - cent) + cent;
		normal *= theta;
		this->knot.assign(cent.data(), cent.data() + 3);
		this->dist = (begPos - cent).norm() * theta;
		this->dir.assign(normal.data(), normal.data() + 3);

	}

	// 欧拉角转换到相对运动: beg -> mid -> end
	auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(beg[3], beg[4], beg[5]);
	auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(mid[3], mid[4], mid[5]);
	auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(end[3], end[4], end[5]);
	// 欧拉角相对值
	auto relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		// 修正欧拉角
		curTraj.mainPoint[3 + i] = begEuler[i] + relEuler[i];
		curTraj.auxPoint[3 + i] = begEuler[i] + relEuler[i] / 2;
		//curTraj.auxPoint[3 + i] = midEuler[i];
	}
	trajList.begin()->mainPoint = curTraj.mainPoint;
	trajList.begin()->auxPoint = curTraj.auxPoint;

	return 0;

}


std::vector<DT_scale> DiscreteTrajectory::get_relative_distance() {

	auto curTraj = get_curTraj();
	auto preTraj = get_preTraj();

	auto curPoint = curTraj.mainPoint;
	auto prePoint = preTraj.mainPoint;
	auto midPoint = curTraj.auxPoint;

	int num = curPoint.size();
	std::vector<DT_scale> relEndMove(num, 0);
	// 终点到起点的相对运动
	for (int i = 0; i < num; ++i) {
		relEndMove[i] = curPoint[i] - prePoint[i];
	}

	// 欧拉角转换到相对运动: beg -> mid -> end
	auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(prePoint[3], prePoint[4], prePoint[5]);
	auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(midPoint[3], midPoint[4], midPoint[5]);
	auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(curPoint[3], curPoint[4], curPoint[5]);
	// 欧拉角相对值
	auto relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		relEndMove[3 + i] = relEuler[i];
	}

	return relEndMove;
}


DiscreteTrajectory::DiscreteTrajectory() {
}


int DiscreteTrajectory::moveJABS(const std::vector<DT_scale>& end, const TrajectoryConfig& config) {

	// 设定轨迹点
	TrajectoryPoint trajPoint;
	trajPoint.mainPoint = end;
	trajPoint.trajType = TrajType::Joint;

	// 定义新轨迹
	SingleTrajectory traj(trajPoint, config);
	traj.saveSeq = trajList.size();

	// 添加轨迹
	trajList.push_back(traj);
	traj.saveSeq = trajList.size();

	return 0;
}


int DiscreteTrajectory::moveLABS(const std::vector<DT_scale>& end, const TrajectoryConfig& config) {

	TrajectoryPoint trajPoint;
	trajPoint.mainPoint = end;
	trajPoint.auxPoint = end;

	if (config.get_moveInBase()) {
		trajPoint.trajType = TrajType::Line_R;

		// 屏蔽地轨指令
		if (trajPoint.mainPoint.size() > 6) {
			trajPoint.mainPoint.erase(trajPoint.mainPoint.begin() + 6, trajPoint.mainPoint.end());
			trajPoint.auxPoint.erase(trajPoint.auxPoint.begin() + 6, trajPoint.auxPoint.end());
		}
	}
	else {
		trajPoint.trajType = TrajType::Line;
	}

	SingleTrajectory traj(trajPoint, config);
	traj.saveSeq = trajList.size();

	trajList.push_back(traj);

	return 0;

}


int DiscreteTrajectory::moveCABS(const std::vector<DT_scale>& mid, const std::vector<DT_scale>& end, const TrajectoryConfig& config) {

	TrajectoryPoint trajPoint;
	trajPoint.mainPoint = end;
	trajPoint.auxPoint = mid;

	if (config.get_moveInBase()) {
		trajPoint.trajType = TrajType::Arc_R;

		// 屏蔽地轨指令
		if (trajPoint.mainPoint.size() > 6) {
			trajPoint.mainPoint.erase(trajPoint.mainPoint.begin() + 6, trajPoint.mainPoint.end());
			trajPoint.auxPoint.erase(trajPoint.auxPoint.begin() + 6, trajPoint.auxPoint.end());
		}
	}
	else {
		trajPoint.trajType = TrajType::Arc;
	}

	SingleTrajectory traj(trajPoint, config);
	traj.saveSeq = trajList.size();

	trajList.push_back(traj);

	return 0;

}


int DiscreteTrajectory::push_new_trajectory(const DiscreteTrajectory& newTraj) {

	trajList.insert(trajList.end(), newTraj.trajList.begin(), newTraj.trajList.end());

	return 0;
}


int DiscreteTrajectory::apply_rotate(const Eigen::Matrix<DT_scale, 3, 3>& rotMat) {

	for (auto& traj : trajList) {

		// 非本体运动，笛卡尔运动
		if (!traj.isJoint() && !traj.isBaseMotion()) {
			// 当前姿态
			auto curEuler = std::vector<DT_scale>(traj.mainPoint.begin() + 3, traj.mainPoint.begin() + 6);

			Eigen::Matrix3f curRotMat = Eigen::AngleAxisf(curEuler[2] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
				Eigen::AngleAxisf(curEuler[1] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(curEuler[0] * DT_PI / 180, Eigen::Vector3f::UnitX()).matrix();

			// 旋转后姿态
			Eigen::Matrix3f aftRotMat = rotMat * curRotMat;

			// 转换为欧拉角
			auto aftEuler = aftRotMat.eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				traj.mainPoint[3 + i] = aftEuler[2 - i] * 180 / DT_PI;
			}

			curEuler = std::vector<DT_scale>(traj.auxPoint.begin() + 3, traj.auxPoint.begin() + 6);
			curRotMat = Eigen::AngleAxisf(curEuler[2] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
				Eigen::AngleAxisf(curEuler[1] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(curEuler[0] * DT_PI / 180, Eigen::Vector3f::UnitX()).matrix();

			// 旋转后姿态
			aftRotMat = rotMat * curRotMat;

			// 转换为欧拉角
			aftEuler = aftRotMat.eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				traj.auxPoint[3 + i] = aftEuler[2 - i] * 180 / DT_PI;
			}
		}
	}

	return 0;
}


bool DiscreteTrajectory::trajectory_loaded() {
	return trajList.empty();
}
// 迭代到下一条轨迹
int DiscreteTrajectory::next() {

	if (trajList.empty()) {
		return 1;
	}
	else {
		preTraj = trajList.front();
		preTraj.lineNum = trajList.front().lineNum;
		trajList.pop_front();
	}

	return 0;
}
void DiscreteTrajectory::clear() {
	trajList.clear();
}
bool DiscreteTrajectory::atLast() const {
	return std::next(trajList.begin()) == trajList.end();
}
SingleTrajectory DiscreteTrajectory::get_curTraj() const {
	return trajList.front();
}

SingleTrajectory DiscreteTrajectory::get_preTraj() const {
	return preTraj;
}

SingleTrajectory DiscreteTrajectory::get_aftTraj() const {
	return (*std::next(trajList.begin()));
}

int DiscreteTrajectory::set_preTraj(const SingleTrajectory& traj) {

	// 当前储存的前一条轨迹与给定轨迹类型相同
	//if (preTraj.trajType == traj.trajType) {
	//}
	// 当前储存的前一条轨迹与给定轨迹不均为空间运动，无需设置
	//if (trajList.front().isCartesian() ^ traj.isCartesian()) {
	//	return 1;
	//}

	preTraj = traj;
	return 0;
}

int DiscreteTrajectory::set_preTraj(const TrajectoryPoint& point) {

	preTraj.set_point(point);

	return 0;

}

bool make_circle(const std::vector<Eigen::Matrix<DT_scale, 3, 1>>& pts, Eigen::Matrix<DT_scale, 3, 1>& center) {
	
	double r = 0.0;
	auto p1 = pts[0];
	auto p2 = pts[1];
	auto p3 = pts[2];
	Eigen::Matrix<DT_scale, 3, 1> v1 = p2 - p1;
	Eigen::Matrix<DT_scale, 3, 1> v2 = p3 - p2;
	Eigen::Matrix<DT_scale, 3, 1> v3 = p3 - p1;

	double n1 = v1.norm();
	v1.normalize();
	double n2 = v2.norm();
	v2.normalize();
	double n3 = v3.norm();
	v3.normalize();
	if (n1 < 1e-2 || n2 < 1e-2 || n3 < 1e-2) {
		return false;
	}
	double cost = (n1 * n1 + n3 * n3 - n2 * n2) / (2 * n1*n3);
	cost = cost<-1.0 ? -1.0 : cost>1.0 ? 1.0 : cost;//圆周角*2=圆心角
	r = sqrt(n2*n2*0.25 / (1 - cost * cost)); //等腰三角形，即0.5*圆心角

	Eigen::Matrix<DT_scale, 3, 1> mp = (p2 + p3)*0.5;
	Eigen::Matrix<DT_scale, 3, 1> norm = v1.cross(v2);
	norm.normalize();
	Eigen::Matrix<DT_scale, 3, 1> xv = v2.cross(norm);
	double dt = r * cost;
	Eigen::Matrix<DT_scale, 3, 1> c1 = mp + xv * dt;
	Eigen::Matrix<DT_scale, 3, 1> c2 = mp - xv * dt;
	center = c1;

	//if (fabs(c2.getDist(p1) - r) < 0.01)
	//	center = c2;
	//double cosd = (n2*n2 + n3 * n3 - n1 * n1) / (2 * n2*n3);
	//cosd = cosd<-1.0 ? -1.0 : cosd>1.0 ? 1.0 : cosd;
	//d1 = acos(cosd) * 2.0;
	//d2 = acos(cost) * 2.0;
	return true;
}

//} // namespace RobotTrajectory
