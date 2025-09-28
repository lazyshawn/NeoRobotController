
#include "robot_interface/ZMotionRobot.h"

#include "RobotLogger.h"

namespace FSAIRobotInterface {

//! 获取下发指令轴号，主要用于确定运动主轴
std::vector<int> ZMotionRobot::get_execute_axis() {
	int base = robotId * 32;
	std::vector<int> axis = { base + 18,base + 19,base + 20,base + 21,base + 22,base + 23 };
	return axis;
}

// 机器人状态
int ZMotionRobot::update_rt_robot_status() {

	int stateIdxBase = get_state_idx_base();
	RobotStatus tmp;
	// !减少读取次数，优化读取速度
	std::vector<float> value;
	//std::vector<int> idx(50, stateIdxBase);
	//for (size_t i = 0; i < idx.size(); ++i) {
	//	idx[i] += i;
	//}
	//ZController->get_axis_param(idx, "TABLE", value);
	ZController->get_register(stateIdxBase, 50, value);

	// 运动状态
	tmp.lowerStatus = static_cast<int>(value[0]);
	// 手动/自动模式
	tmp.autoMode = static_cast<int>(value[1]);
	// 正逆解模式
	tmp.fkMode = static_cast<int>(value[2]);
	// 运动行号
	tmp.lineNum = static_cast<int>(value[3]);
	// 轨迹指令编号
	tmp.cmdNum = static_cast<int>(value[7]);
	// 关节位置
	tmp.jPos = std::vector<float>(value.begin() + 10, value.begin() + 16);
	tmp.jPos.insert(tmp.jPos.end(), value.begin() + 22, value.begin() + 25);
	// 空间位置
	tmp.cPosRaw = std::vector<float>(value.begin() + 16, value.begin() + 22);
	tmp.cPosRaw.insert(tmp.cPosRaw.end(), value.begin() + 22, value.begin() + 25);

	tmp.remainBuffer = static_cast<int>(value[5]);

	// 当前时间戳
	tmp.slaveTime = static_cast<long>(value[28]);
	// 主轴运动距离
	tmp.masterAxisDist = static_cast<float>(value[29]);

	// 电流
	tmp.current = value[33];
	// 电压
	tmp.voltage = value[34];

	// 焊接总时长
	tmp.weldTime = static_cast<long>(value[30]);
	// 起弧时间
	tmp.weldBegTime = static_cast<long>(value[31]);
	// 息弧时间
	tmp.weldEndTime = static_cast<long>(value[32]);

	// 局部坐标系转世界坐标系
	tmp.cPos = tmp.cPosRaw;
	cpos_base_to_world(tmp.cPos);

	// 加锁
	std::lock_guard<std::mutex> lock(mtx);
	// 需要保持的状态
	tmp.upperStatus = robotStatus.upperStatus;

	robotStatus = tmp;

	return 0;
}

int ZMotionRobot::get_all_robot_status(RobotStatus& status) {
	{
		// 加锁
		std::lock_guard<std::mutex> lock(mtx);

		// 更新机器人状态
		status = robotStatus;
	}

	// 轴号
	std::vector<int> axis;
	// 读取非实时参数
	std::vector<float> value;
	// 机器人坐标系
	axis = get_composed_axis({ get_robot_tcp_axis(), robotConfig.appAxisIdxRead });
	ZController->get_axis_param(axis, "DPOS", status.cPosR);
	// 编码器值
	axis = get_composed_axis({ get_axis_idx(), robotConfig.appAxisIdxRead });
	ZController->get_axis_param(axis, "ENCODER", value);
	status.encoder = std::vector<int>(value.size(), 0);
	for (size_t i = 0; i < axis.size(); ++i) {
		status.encoder[i] = static_cast<int>(value[i]);
	}
	// 轴状态
	ZController->get_axis_param(axis, "AXISSTATUS", value);
	status.axisStatus = std::vector<int>(value.size(), 0);
	for (size_t i = 0; i < axis.size(); ++i) {
		status.axisStatus[i] = static_cast<int>(value[i]);
	}

	return 0;
}


int ZMotionRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
	return 0;
}
int ZMotionRobot::moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
	int num = (std::min)(beg.size(), end.size());
	std::vector<float> rel(num, 0.0);
	for (size_t i = 0; i < num; ++i) {
		rel[i] = end[i] - beg[i];
	}

	return ZController->move(axis, rel, 1, mask);
}

int ZMotionRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(relMove.size(), axis.size());

	return ZController->move(axis, relMove, 0, mask);

}
int ZMotionRobot::moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(beg.size(), axis.size());
	int ret = 0;

	// 计算相对值
	auto relEndMove = get_relative_distance(beg, end, end);

	return ZController->move(axis, relEndMove, 0, mask);
}

int ZMotionRobot::moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
	return 0;
}
int ZMotionRobot::moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {

	size_t num = (std::min)(beg.size(), axis.size());
	num = (std::min)(num, mid.size());

	// 中间点相对值
	std::vector<float> relMidMove(num);
	for (size_t i = 0; i < num; ++i) {
		relMidMove[i] = mid[i] - beg[i];
	}

	// 终点相对值
	std::vector<float> relEndMove(num);
	for (size_t i = 0; i < num; ++i) {
		relEndMove[i] = end[i] - beg[i];
	}

	// 欧拉角转换到相对运动: beg -> mid -> end
	auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(beg[3], beg[4], beg[5]);
	auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(mid[3], mid[4], mid[5]);
	auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(end[3], end[4], end[5]);
	// 欧拉角相对值
	auto relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		// 修正欧拉角
		relEndMove[3 + i] = relEuler[i];
	}

	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num - 1; i++) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}

		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)", axis[num - 1]);
	strcat(cmdbuff, tempbuff);
	strcat(cmdbuff, "\n");

	sprintf(tempbuff, "MSPHERICALSP(%f,%f,%f,%f,%f,%f,%d", relEndMove[0], relEndMove[1], relEndMove[2], relMidMove[0], relMidMove[1], relMidMove[2], imode);
	strcat(cmdbuff, tempbuff);
	for (size_t i = 3; i < num; ++i) {
		// 轴屏蔽
		if (mask.size() > i && mask[i] <= 0) {
			continue;
		}

		sprintf(tempbuff, ",%f", relEndMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")");

	//调用命令执行函数
	return ZController->sendCmd(cmdbuff, cmdbuffAck);
}

int ZMotionRobot::set_manual_speed(float ratio) {

	int stateIdxBase = get_state_idx_base();
	if (robotStatus.autoMode > 0) {
		return 1;
	}

	// 保存到 table, 触发速度刷新
	ZController->set_axis_param({ stateIdxBase + 4, stateIdxBase + 53 }, "TABLE", { static_cast<float>(ratio / 100.0), 1.0 });

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " set speed ratio: " << ratio / 100.0);

	return 0;
}

// 自动任务
int ZMotionRobot::update_swing_config() {

	int ret;
	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();

	// 获取节点目标位置
	auto curPoint = curTraj.mainPoint;
	auto prePoint = preTraj.mainPoint;
	auto midPoint = curTraj.auxPoint;

	// 读取焊接参数
	Weave waveCfg = deserialize_Weave(curTraj.get_appendix());

	bool sendPlainTraj = false;
	// 摆焊开启
	if (waveCfg.Id > 0) {
		// 正弦摆
		if (waveCfg.Shape == 0) {
			// 修改摆动参数
			int numPeriod = get_swing_num();
			// 停留1ms，减小抖动
			ZController->set_base_param(get_execute_axis()[0], "MOVE_WA", { 1.0 });

			// 摆焊周期数大于0
			if (numPeriod > 0) {
				// 计算旋转平面方向
				auto trajInfo = calc_traj_info(prePoint, midPoint, curPoint, curTraj.isArc());
				std::vector<float> nDir = { trajInfo[4], trajInfo[5], trajInfo[6] };
				double norm = std::sqrt(nDir[0] * nDir[0] + nDir[1] * nDir[1] + nDir[2] * nDir[2]);
				for (size_t i = 0; i < nDir.size(); ++i) {
					nDir[i] /= norm;
				}

				// 点位转到世界坐标系
				auto wCPos = prePoint;
				cpos_base_to_world(wCPos);

				// 计算摆焊方向
				std::vector<float> zDir(3), zEuler = { static_cast<float>(wCPos[3] * DT_PI / 180),
					static_cast<float>(wCPos[4] * DT_PI / 180), static_cast<float>(wCPos[5] * DT_PI / 180) };

				zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
				zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
				zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

				// 设置摆焊
				update_swing_table(waveCfg);

				//ZController->set_base_param(get_execute_axis()[0], "MOVE_WA", { 1.0 });

				int swingMode = curTraj.isLine() ? 3 : 5;
				//ret = swing_on((trajectory.get_dist() - 0.02) / numPeriod, waveCfg, swingMode, zDir, nDir);
				ret = swing_on((trajectory.get_dist() - 0.00) / numPeriod, waveCfg, swingMode, zDir, nDir);

				// 计算轴运动距离
				//ret += swing_off(trajectory.get_dist() - 0.02);

			}

		}
		// 三角摆
		else if (waveCfg.Shape == 3) {

		}
	}

	if (waveCfg.Id > 0 && !sendPlainTraj) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Update swing config: " <<
			vector_to_string(serialize_Weave(waveCfg).second, 2)
		);
	}

	return 0;
}

int ZMotionRobot::update_track_config() {

	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();
	Track trackCfg = deserialize_Track(curTraj.get_appendix());

	int stateIdxBase = get_state_idx_base();
	std::vector<int> axis = get_composed_axis({ get_execute_axis(), robotConfig.appAxisIdx });
	std::vector<float> config = serialize_Track(trackCfg).second;
	size_t configTableStart = stateIdxBase + 180;
	int ret = 0;

	// 跟踪未使能
	if (trackCfg.Id > 0) {
		// 电弧跟踪标志位，区分电弧跟踪和线激光跟踪
		ZController->set_axis_param(stateIdxBase + 150, "TABLE", trackCfg.Id, axis[0]);

		// 下发跟踪参数
		for (size_t i = 0; i < config.size(); ++i) {
			ZController->set_axis_param(configTableStart + i, "TABLE", config[i], axis[0]);
		}

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId <<
			" Update track config: " << vector_to_string(serialize_Track(trackCfg).second, 2)
		);
	}
	else {
		ZController->set_axis_param(stateIdxBase + 187, "TABLE", trackCfg.Id, axis[0]);
	}

	return 0;
}
int ZMotionRobot::update_welder_config() {

	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();
	Arc_WeldingParaItem weldCfg = deserialize_Arc_WeldingParaItem(curTraj.get_appendix());

	// 不起弧，无需修改焊接参数
	if (weldCfg.Id <= 0) {
		return 1;
	}

	float current, voltage;
	// 电流
	current = weldCfg.WeldingCrt_Spd;
	// 电压分别模式
	//if (weldCfg.WeldingWorkMode == 4) {
	if ((weldCfg.WeldingWorkMode >> 4) % 2 == 1) {
		voltage = weldCfg.WeldingVtg_Strth;
	}
	// 一元模式
	else {
		voltage = weldCfg.VtgUniCorrection + 30;
	}

	uint8_t modeCmd = 0;
	//modeCmd += (1 << 1);
	//modeCmd += (weldCfg.WeldingWorkMode == 1) << 2;
	modeCmd += weldCfg.WeldingWorkMode;
	modeCmd += (weldCfg.Id >= 0);

	int stateBase = get_state_idx_base();
	std::vector<int> tableList(3, stateBase + 171);
	for (size_t i = 0; i < tableList.size(); ++i) {
		tableList[i] += i;
	}

	std::vector<float> data;
	data.push_back(modeCmd);
	data.push_back(current);
	data.push_back(voltage);

	// 写入变工艺参数
	ZController->set_axis_param(tableList, "TABLE", data, get_execute_axis()[0]);
	// 变工艺使能
	ZController->set_axis_param(stateBase + 170, "TABLE", 1, get_execute_axis()[0]);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId <<
		" Update welder config: " << vector_to_string(serialize_Arc_WeldingParaItem(weldCfg).second, 2)
	);

	return 0;
}
int ZMotionRobot::get_remain_buffer() {
	int idx = get_state_idx_base() + 5;
	float value;

	ZController->get_axis_param(idx, "TABLE", value);

	return static_cast<int>(value);
}

int ZMotionRobot::push_new_trajectory(DiscreteTrajectory trajList) {

	// 未设置自动模式
	if (robotStatus.autoMode <= 0) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " switch to auto mode before push trajectory.");
		set_upperStatus(0x10);
		return -1;
	}

	// 轨迹为空
	if (trajList.size() == 0) {
		return -2;
	}

	// 轨迹预处理
	// 按主从标定矩阵，将世界坐标系姿态转换为基坐标系姿态
	auto rotMat = robotConfig.get_slave_calibratino_mat().inverse();
	trajList.apply_rotate(rotMat);

	std::unique_lock<std::mutex> lock(mtx);
	// 等待条件置反
	motionDone = false;

	// 轨迹入栈
	trajectory.push_new_trajectory(trajList);
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " receive new trajectory, trajectory buffer size is " << trajectory.size());

	return 0;

}


int ZMotionRobot::execute_single_joint() {
	int ret = 0;
	// 前一条轨迹
	auto preTraj = trajectory.get_preTraj();
	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();

	std::vector<int> axis = get_composed_axis({ get_execute_axis(), robotConfig.appAxisIdx });

	// 轴屏蔽
	std::vector<int> mask(axis.size(), 1);
	auto trajAxisMask = curTraj.get_axisMask();
	for (size_t i = 0; i < axis.size(); ++i) {
		// 机器人指定的屏蔽
		if (axisMask.count(i) > 0)
			mask[i] = -1;
		// 轨迹指定的屏蔽
		for (size_t j = 0; j < trajAxisMask.size(); ++j) {
			if (i == trajAxisMask[j])
				mask[i] = -1;
		}
	}
	std::vector<float> maskF;
	for (size_t i = 0; i < mask.size(); ++i) {
		if (mask[i] < 0)
			maskF.push_back(static_cast<float>(axis[i]));
	}

	// 速度值
	std::vector<float> speed(axis.size(), 1);

	// 运动类型检查

	// 获取节点目标位置
	auto pnt = curTraj.mainPoint;

	// 设置速度
	float speedRatio = curTraj.get_speed() / 100.0;
	if (speedRatio > 0) {
		for (int i = 0; i < axis.size(); ++i) {
			speed[i] = robotConfig.maxJointSpeedAuto[i] * speedRatio;
		}
		ZController->set_axis_param(axis, (char*)"SPEED", speed);
	}
	// 设置平滑度
	if (curTraj.get_smooth() >= 0) {
		ZController->set_axis_param(axis[0], (char*)"ZSMOOTH", curTraj.get_smooth());
	}

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveJABS: " << vector_to_string(pnt));
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
		<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
		<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : ""));

	// 开始记录位置
	save_task_status(true, axis[0]);
	// 下发轨迹编号
	send_running_line_num(axis[0], trajectory.get_curTraj());

	// 获取前一条轨迹位置
	auto beg = preTraj.get_mainPoint();
	// 下发轨迹
	ret = moveJABS(axis, beg, pnt, mask);
	if (ret != 0)
		return ret;

	// 更新轨迹编号
	trajectory.trajList.front().lineNum = ++cmdNum;

	// 下发轨迹序号
	send_line_num(axis[0], trajectory.get_curTraj());
	// 停止记录位置
	save_task_status(false, axis[0]);

	// 轨迹出栈
	if (ret == 0) {
		trajectory.set_current_line_num(cmdNum);
		trajectory.next();
	}
	else {
		return -1;
	}

	return 0;
}

int ZMotionRobot::execute_single_cartesian() {

	int stateIdxBase = get_state_idx_base();
	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();

	int ret = 0;
	std::vector<int> axis = get_composed_axis({ get_execute_axis(), robotConfig.appAxisIdx });
	std::vector<int> camAxis = get_cam_axis();

	// 轴屏蔽
	std::vector<int> mask(axis.size(), 1);
	auto trajAxisMask = curTraj.get_axisMask();
	for (size_t i = 0; i < axis.size(); ++i) {
		if (axisMask.count(i) > 0)
			mask[i] = -1;
		for (size_t j = 0; j < trajAxisMask.size(); ++j) {
			if (i == trajAxisMask[j])
				mask[i] = -1;
		}
	}
	std::vector<float> maskF;
	for (size_t i = 0; i < mask.size(); ++i) {
		if (mask[i] < 0)
			maskF.push_back(static_cast<float>(axis[i]));
	}

	// 获取节点目标位置
	auto curPoint = curTraj.mainPoint;
	auto prePoint = preTraj.mainPoint;
	auto midPoint = curTraj.auxPoint;

	// 计算姿态变化
	Eigen::Quaternionf preOri = Eigen::AngleAxisf(prePoint[5] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(prePoint[4] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(prePoint[3] * DT_PI / 180, Eigen::Vector3f::UnitX());
	Eigen::Quaternionf curOri = Eigen::AngleAxisf(curPoint[5] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(curPoint[4] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(curPoint[3] * DT_PI / 180, Eigen::Vector3f::UnitX());
	float detOri = preOri.angularDistance(curOri) * 180 / DT_PI;
	// 假定最大45 deg/s
	float oriTime = detOri / 45;
	// 速度修正
	float cartTime = trajectory.get_dist() / curTraj.get_speed();
	float correctSpeed = -1.0;
	if (oriTime > cartTime) {
		correctSpeed = curTraj.get_speed() * oriTime / cartTime;
	}

	if (curTraj.isArc()) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveCABS: " << vector_to_string(curPoint)
			<< ";\nmid: " << vector_to_string(midPoint)
		);
	}
	else {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveLABS: " << vector_to_string(curPoint));
	}
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
		<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
		<< (correctSpeed > 0 ? ", correct: " + std::to_string(correctSpeed) : "")    // 速度修正
		<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : "")     // 轴掩码
		<< ". traj dist: " << trajectory.get_dist() << ", " << detOri
	);

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(curPoint.size(), axis.size());

	// 读取焊接参数
	Weave waveCfg = deserialize_Weave(curTraj.get_appendix());
	Arc_WeldingParaItem weldCfg = deserialize_Arc_WeldingParaItem(curTraj.get_appendix());
	Track trackCfg = deserialize_Track(curTraj.get_appendix());

	// 修改焊接参数
	update_welder_config();
	// 修改跟踪参数
	update_track_config();
	// 修改摆焊参数
	update_swing_config();

	// 设置平滑度
	if (curTraj.get_smooth() >= 0)
		ZController->set_axis_param(axis[0], "ZSMOOTH", curTraj.get_smooth());
	// 设置速度
	ZController->set_axis_param(axis[0], "FORCE_SPEED", correctSpeed > 0 ? correctSpeed : curTraj.get_speed());
	// 修改加速度
	Weave preWaveCfg = deserialize_Weave(preTraj.get_appendix());
	// 前一条不摆焊，当前摆焊，修改加速度
	if (preWaveCfg.Id <= 0 && waveCfg.Id > 0) {
		execute_move_action({ { 7, { 1 } } }, 0);
	}
	else if (preWaveCfg.Id > 0 && waveCfg.Id <= 0) {
		execute_move_action({ { 7, { 2 } } }, 0);
	}

	// 开始记录位置
	save_task_status(true, axis[0]);
	// 下发轨迹编号
	send_running_line_num(axis[0], trajectory.get_curTraj());


	// 当前轨迹的相对运动量
	std::vector<float> relEndMove = trajectory.get_relative_distance();

	// 开启摆焊
	int numPeriod = get_swing_num();
	if (waveCfg.Id > 0 && numPeriod > 0) {
		// 正弦摆
		if (waveCfg.Shape == 0) {
			// 第一个1/4周期占用的相位角
			float detQ = std::asin((waveCfg.LeftWidth - waveCfg.RightWidth) / (waveCfg.LeftWidth + waveCfg.RightWidth));
			float rightPartial = 1.0 / numPeriod * (DT_PI / 2 - detQ) / (2 * DT_PI);
			float leftPartial = 1.0 / numPeriod * (DT_PI / 2 + detQ) / (2 * DT_PI);

			float begPartial = 0, endPartial = 0;
			auto segmentBeg = preTraj.mainPoint;
			for (size_t i = 0; i < numPeriod; ++i) {
				// 向右1/4
				begPartial = endPartial;
				endPartial += rightPartial;
				auto segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);
				if (curTraj.isArc())
					ret = moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					ret = moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				segmentBeg = segment.mainPoint;
				begPartial = endPartial;
				if (waveCfg.Dwell_right > 0)
					ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_right) });
				if (ret != 0)
					return ret;

				// 右1/4
				begPartial = endPartial;
				endPartial += rightPartial;
				segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);
				if (curTraj.isArc())
					ret = moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					ret = moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				segmentBeg = segment.mainPoint;
				if (waveCfg.Dwell_center > 0)
					ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_center) });
				if (ret != 0)
					return ret;

				// 记录电流
				if (/*i > 0 && */i < numPeriod - 1)
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 2, axis[0]);
				else
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 1, axis[0]);

				// 右侧触发跟踪
				if (i > 0 && i < numPeriod - 1)
					ZController->set_axis_param(stateIdxBase + 160, "TABLE", 2, axis[0]);

				// 左1/4
				begPartial = endPartial;
				endPartial += leftPartial;
				segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);
				if (curTraj.isArc())
					ret = moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					ret = moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				segmentBeg = segment.mainPoint;
				if (waveCfg.Dwell_left > 0)
					ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_left) });
				if (ret != 0)
					return ret;

				// 左1/4
				begPartial = endPartial;
				endPartial += leftPartial;
				segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);
				if (curTraj.isArc())
					ret = moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					ret = moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				segmentBeg = segment.mainPoint;
				if (waveCfg.Dwell_center > 0 && i < numPeriod - 1)
					ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_center) });
				if (ret != 0)
					return ret;

				// 记录电流
				if (/*i > 0 && */i < numPeriod - 1)
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 3, axis[0]);
				else
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 1, axis[0]);

				// 左侧触发跟踪
				if (i > 0 && i < numPeriod - 1)
					ZController->set_axis_param(stateIdxBase + 160, "TABLE", 3, axis[0]);

				// 记录多层多道点位
				if (i == numPeriod / 2) {
					ZController->set_axis_param(stateIdxBase + 101, "TABLE", (curTraj.isLine() ? 1 : 2), axis[0]);
				}
				else if (i == numPeriod - 1) {
					ZController->set_axis_param(stateIdxBase + 101, "TABLE", 3, axis[0]);
				}
			}

			// 停止以防速度突变
			//ZController->set_base_param(axis[0], "MOVE_WA", { 1.0 });
			// 摆焊结束
			ret += swing_off(trajectory.get_dist() - 0.02);
		}
		else if (waveCfg.Shape == 3) {
			// 加入摆动轴
			axis.insert(axis.end(), camAxis.begin(), camAxis.end());

			// 三角摆: 摆幅、前进比例、后退比例
			float triWidth = waveCfg.LeftWidth + waveCfg.RightWidth, feedRatio = waveCfg.Length, backRatio = waveCfg.Bias;
			// 横移速度
			float shiftVel = triWidth / 2 * waveCfg.Freq;
			// 焊接速度
			float weldVel = curTraj.get_speed();
			// 前进距离
			float feedDist = weldVel / shiftVel * triWidth / 2 * feedRatio;
			// 完整的三角摆周期
			int numPeriod = std::ceil((trajectory.get_dist()/* - feedDist*/) / (feedDist * (feedRatio - backRatio) / feedRatio));
			// 前进距离修正
			feedDist = trajectory.get_dist() / (numPeriod*(feedRatio - backRatio) / feedRatio/* + 1*/);
			// 后退距离
			float backDist = feedDist * backRatio / feedRatio;
			// 实际进给距离，回退距离
			float actFeedDist = sqrt(feedDist * feedDist + triWidth * triWidth / 4), actBackDist = sqrt(backDist * backDist + triWidth * triWidth / 4);
			// 前进时间, 后退时间
			float feedTime = actFeedDist / weldVel, backTime = actBackDist / weldVel;
			// 左右摆角
			float rightAngle = waveCfg.Angle_Ltype_top, leftAngle = waveCfg.Angle_Ltype_btm;

			// 轨迹段起点处的欧拉角(deg)
			Eigen::Vector3f zEuler(prePoint[3] * DT_PI / 180, prePoint[4] * DT_PI / 180, prePoint[5] * DT_PI / 180);
			// 缓冲最终位置的工具 Z 方向
			Eigen::Vector3f zDir(0, 0, 0);
			zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
			zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
			zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

			// 起点切线方向
			Eigen::Vector3f begTan(trajectory.get_dir()[0], trajectory.get_dir()[1], trajectory.get_dir()[2]);
			// 修正的z方向
			Eigen::Vector3f begUprightDir = (begTan.cross(zDir).cross(begTan)).normalized();
			// 左右摆动方向: 暂时用0
			rightAngle = 0;
			leftAngle = 0;
			Eigen::Vector3f rightDir = Eigen::AngleAxisf(rightAngle*DT_PI / 180 - DT_PI / 2, begTan) * begUprightDir;
			Eigen::Vector3f leftDir = Eigen::AngleAxisf(leftAngle*DT_PI / 180 + DT_PI / 2, begTan) * begUprightDir;


			std::vector<float> offsetCmd(3, 0);
			float begPartial = 0.0, endPartial = 0.0;
			auto segmentBeg = preTraj.mainPoint;
			auto segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 1);
			for (size_t j = segment.mainPoint.size(); j < axis.size() - 3; ++j) {
				segment.mainPoint.push_back(0);
				segment.auxPoint.push_back(0);
			}
			segment.mainPoint.insert(segment.mainPoint.end(), offsetCmd.begin(), offsetCmd.end());
			segmentBeg = segment.mainPoint;

			ZController->set_axis_param(axis[0], "FORCE_SPEED", shiftVel);
			for (size_t i = 0; i < numPeriod; ++i) {

				// 右侧向右
				// 偏移
				for (size_t j = 0; j < 3; ++j)
					segment.mainPoint[segment.mainPoint.size() - 3 + j] = rightDir[j] * waveCfg.RightWidth;
				// 下发指令
				if (curTraj.isArc())
					moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				// 到达右侧
				//ZController->set_axis_param(stateIdxBase + 151, "TABLE", 1, axis[0]);
				ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_right) });
				segmentBeg = segment.mainPoint;

				// 右侧向前 + 姿态
				endPartial += feedDist;
				segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 1);
				for (size_t j = 0; j < 3; ++j)
					offsetCmd[j] = 0;
				for (size_t j = segment.mainPoint.size(); j < axis.size() - 3; ++j) {
					segment.mainPoint.push_back(0);
					segment.auxPoint.push_back(0);
				}
				segment.mainPoint.insert(segment.mainPoint.end(), offsetCmd.begin(), offsetCmd.end());
				ZController->set_axis_param(axis[0], "FORCE_SPEED", weldVel);
				if (curTraj.isArc())
					moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_center) });
				begPartial = endPartial;
				segmentBeg = segment.mainPoint;

				// 记录电流
				if (i > 0 && i < numPeriod - 1) {
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 2, axis[0]);
					// 触发跟踪
					ZController->set_axis_param(stateIdxBase + 160, "TABLE", 2, axis[0]);
				}
				else
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 1, axis[0]);


				// 左测向后
				endPartial -= backDist;
				segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 1);
				// 只修改位置部分
				for (size_t j = 3; j < segment.mainPoint.size(); ++j) {
					segment.mainPoint[j] = segmentBeg[j];
				}
				for (size_t j = segment.mainPoint.size(); j < axis.size() - 3; ++j) {
					segment.mainPoint.push_back(0);
					segment.auxPoint.push_back(0);
				}
				for (size_t j = 0; j < 3; ++j)
					offsetCmd[j] = leftDir[j] * waveCfg.LeftWidth;
				segment.mainPoint.insert(segment.mainPoint.end(), offsetCmd.begin(), offsetCmd.end());
				if (curTraj.isArc())
					moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				// 到达左侧
				//ZController->set_axis_param(stateIdxBase + 151, "TABLE", -1, axis[0]);
				ZController->set_base_param(axis[0], "MOVE_WA", { static_cast<float>(waveCfg.Dwell_left) });
				begPartial = endPartial;
				segmentBeg = segment.mainPoint;

				// 左侧向右
				for (size_t j = 0; j < 3; ++j) {
					segment.mainPoint[segment.mainPoint.size() - 3 + j] = 0;
				}
				ZController->set_axis_param(axis[0], "FORCE_SPEED", shiftVel);
				if (curTraj.isArc())
					moveCABS(axis, segmentBeg, segment.auxPoint, segment.mainPoint, 0, mask);
				else
					moveLABS(axis, segmentBeg, segment.mainPoint, mask);
				segmentBeg = segment.mainPoint;

				// 记录电流
				if (i > 0 && i < numPeriod - 1)
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 3, axis[0]);
				else
					ZController->set_axis_param(stateIdxBase + 151, "TABLE", 1, axis[0]);
				// 周期结束
				//ZController->set_axis_param(stateIdxBase + 152, "TABLE", 1, axis[0]);
			}

		}
	}
	// 无摆焊，正常下发
	else {
		if (curTraj.isArc()) {
			moveCABS(axis, prePoint, midPoint, curPoint, 0, mask);
		}
		else if (curTraj.isLine()) {
			moveL(axis, relEndMove, mask);
		}
	}

	// 更新轨迹编号
	trajectory.trajList.front().lineNum = ++cmdNum;

	// 下发轨迹序号
	send_line_num(axis[0], trajectory.get_curTraj());
	// 停止记录位置
	save_task_status(false, axis[0]);


	// 下发异常
	if (ret == 0) {
		//traj.set_current_line_num(cmdNum);
		trajectory.next();
	}
	else {
		return -1;
	}

	return ret;
}

int ZMotionRobot::send_running_line_num(int axis, const SingleTrajectory &curTraj) {
	int stateIdxBase = get_state_idx_base();
	int ret = 0;

	// 下发轨迹编号
	ret = ZController->set_axis_param(stateIdxBase + 7, "TABLE", curTraj.saveSeq, axis);
	// 下发轨迹类型
	ret = ZController->set_axis_param(stateIdxBase + 102, "TABLE", curTraj.isJoint() ? 1 : -1, axis);

	LOG4CPLUS_INFO(RobotLog::getLogger(),
		"R" << aliasId << " ready to send traj num: " << curTraj.saveSeq
	);
	return 0;
}

int ZMotionRobot::send_line_num(int axis, const SingleTrajectory &curTraj) {
	int stateIdxBase = get_state_idx_base();
	int ret = 0;

	ret = ZController->set_axis_param(stateIdxBase + 3, "TABLE", curTraj.lineNum, axis);

	LOG4CPLUS_INFO(RobotLog::getLogger(),
		"R" << aliasId << " send point with line num: " << curTraj.lineNum
	);

	return ret;
}

// 剩余缓冲检测
int ZMotionRobot::remain_buffer_free() {
	// 获取缓存长度
	int remainBuffer = get_remain_buffer();

	return remainBuffer > 1000;
}

// 一致性轨迹预处理，可以连续下发的轨迹
int ZMotionRobot::set_ready_for_consistent_traj(int& state) {

	if (trajectory.trajectory_loaded())
		return 0;

	int ret = 0;
	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();
	bool trajReady = false;

	// 正逆解切换完成
	if (kinematics_mached()) {

		// 当前轨迹与前一条轨迹类型不一致，或前一条轨迹为空，需要记录起点
		if (preTraj.trajType == TrajType::None || curTraj.isJoint() ^ preTraj.isJoint() ||
			(!curTraj.isJoint() && !preTraj.isJoint() && curTraj.isBaseMotion() ^ preTraj.isBaseMotion())) {

			// 下发轨迹编号运动完成
			if (preTraj.lineNum == robotStatus.lineNum && robotStatus.lowerStatus == 0) {
				// 正逆解切换完成，重新设定上条轨迹
				TrajectoryPoint point;
				point.trajType = curTraj.trajType;
				point.mainPoint = curTraj.isJoint() ? robotStatus.jPos : robotStatus.cPosRaw;
				point.auxPoint = robotStatus.cPosRaw;

				trajectory.set_preTraj(point);

				auto printPoint = curTraj.isJoint() ? robotStatus.jPos : robotStatus.cPos;
				LOG4CPLUS_INFO(RobotLog::getLogger(),
					"R" << aliasId << " switch to " << (curTraj.isJoint() ? "forward" : "inverse") << " kinematics"
					<< (curTraj.isBaseMotion() ? " (base)" : "") << ".\n"
					<< "CurPos: " << vector_to_string(printPoint) << "\n"
					<< "RawPos: " << vector_to_string(point.mainPoint)
				);

				// 计算协同段距离
				//calc_sync_duration(robotIdx);
			}

		}

		trajReady = true;
	}

	set_bit(state, 9, trajReady);

	return 0;
}

// 一致性轨迹就绪
int ZMotionRobot::consistent_traj_ready(int& state) {

	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();
	int ret = 0;

	// 正逆解变化: 此处应使用辅助判断，避免检测结果与预处理部分不同，从而引发因时序问题导致下发异常
	if (!kinematics_mached() || !get_bit(state, 9)) {

		int switchRet = 0;
		// 运动未完成，不进行切换
		if (preTraj.lineNum != robotStatus.lineNum || robotStatus.lowerStatus != 0) {
			// 共用轴正在运动，无法切换
			//return -10;
		}
		// 尝试切换正逆解
		else if (curTraj.isJoint()) {
			switchRet = switch_kinematics(1);
		}
		else if (curTraj.isBaseMotion()) {
			switchRet = switch_kinematics(-2);
		}
		else {
			switchRet = switch_kinematics(-1);
		}

		ret++;
	}

	// 第一条正逆解未切换
	if (preTraj.trajType == TrajType::None) {
		if (get_bit(state, 4) == 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " previous trajectory type is NONE.");
			set_bit(state, 4, true);
		}
		ret++;
	}
	else {
		set_bit(state, 4, false);
	}

	return ret > 0 ? 0 : 1;

}

int ZMotionRobot::separate_trajectory() {

	auto ite = trajectory.trajList.begin();

	// 关节轨迹无需分段
	if (ite->isJoint()) {
		return 1;
	}

	// 计算轨迹参数
	trajectory.calc_traj_info();

	auto waveCfg = deserialize_Weave(ite->get_appendix());

	// 无摆焊无需分段
	if (waveCfg.Id <= 0)
		return 1;

	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();

	// 备份需要修改的运动参数
	Move_Action moveCfgBk = deserialize_Move_Action(curTraj.get_appendix());

	// 旋转角度小，用直线近似
	Eigen::Vector3f dir = Eigen::Vector3f(trajectory.get_dir().data());
	if (curTraj.isArc() && dir.norm() < 1e-2) {
		ite->trajType = TrajType::Line;
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " convert to traj Line: " << (dir.norm() * 180 / DT_PI));
		trajectory.calc_traj_info();
	}

	// 计算摆焊段数
	int numPeriod = std::round(trajectory.get_dist() / (curTraj.get_speed() / waveCfg.Freq));

	// 估计占用缓冲数
	int bufferSize = 0;
	if (curTraj.isArc()) {
		bufferSize = 4 * numPeriod * (9 + 2);
	}
	else {
		bufferSize = 4 * numPeriod * (1 + 2);
	}

	// 轨迹分段
	float maxBuffSize = 1000.0;
	if (bufferSize > maxBuffSize) {

		// 轨迹段数，向上取整
		int trajSize = std::ceil(bufferSize / maxBuffSize);

		if (curTraj.isArc()) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " separated traj Arc : " << vector_to_string(curTraj.mainPoint) << ";\n"
				<< "mid: " << vector_to_string(curTraj.auxPoint) << ".\n"
				<< "traj separated to: " << trajSize << ", dist: " << trajectory.get_dist()
			);
		}
		else {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " separated traj Line : " << vector_to_string(curTraj.mainPoint) << ".\n"
				<< "traj separated to: " << trajSize << ", dist: " << trajectory.get_dist()
			);
		}

		float begPartial = 1.0 - 1.0 / trajSize, endPartial = 1.0;
		auto segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);

		auto traj = curTraj;
		traj.mainPoint = segment.mainPoint;
		traj.auxPoint = segment.auxPoint;
		// 清空轨迹前动作
		auto moveCfg = moveCfgBk;
		moveCfg.actionBefore.clear();
		traj.add_appendix(serialize_Move_Action(moveCfg));
		// 修改当前轨迹(最后一段)
		*ite = traj;

		begPartial = 0.0, endPartial = 0.0;
		for (size_t i = 0; i < trajSize - 1; ++i) {

			begPartial = endPartial;
			endPartial += 1.0 / trajSize;

			// 轨迹分段
			segment = partition_trajectory(preTraj.get_point(), curTraj.get_point(), begPartial, endPartial, 0);

			traj = curTraj;
			traj.mainPoint = segment.mainPoint;
			traj.auxPoint = segment.auxPoint;

			auto moveCfg = moveCfgBk;
			// 第一条轨迹
			if (i == 0) {
				// 清空轨迹后动作
				moveCfg.actionAfter.clear();
				traj.add_appendix(serialize_Move_Action(moveCfg));
			}
			else {
				// 清空轨迹动作
				moveCfg.actionBefore.clear();
				moveCfg.actionAfter.clear();
				traj.add_appendix(serialize_Move_Action(moveCfg));
			}

			// 插入新轨迹
			trajectory.trajList.insert(ite, traj);

		}

		// 重新计算轨迹参数
		trajectory.calc_traj_info();

	}

	return 0;
}

/* *************************** 上层自定义接口 *************************** */
int ZMotionRobot::switch_auto(bool enableAuto) {

	int stateIdxBase = get_state_idx_base();
	// 清除模式不匹配的异常
	robotStatus.upperStatus &= 0xEF;

	// 切换手动/自动模式
	ZController->set_axis_param(stateIdxBase + 50, "TABLE", enableAuto ? 1 : -1);

	// 手动模式时切换回正解模式
	//if (!enableAuto) {
	//	switch_kinematics(1);
	//}

	//update_robotStatus();
	RobotStatus tmpStatus;
	get_rt_robot_status(tmpStatus);

	// 检测是否切换成功

	// 设定轨迹起点
	TrajectoryPoint point;
	if (tmpStatus.fkMode >= 0) {
		point.mainPoint = tmpStatus.jPos;
		point.auxPoint = tmpStatus.jPos;
	}
	else {
		point.mainPoint = tmpStatus.cPos;
		point.auxPoint = tmpStatus.cPos;
	}
	//point.trajType = TrajType::None;
	//trajectory.set_preTraj(point);
	LOG4CPLUS_INFO(RobotLog::getLogger(),
		"R" << aliasId << " switch to " << (enableAuto ? "auto" : "manual") << " mode."
		<< " upperStatus: " << robotStatus.upperStatus << ", "
		<< " TrajType: " << static_cast<int>(trajectory.get_preTraj().get_trajType()) << "\n" <<
		"CurPos: " << vector_to_string(point.mainPoint)
	);

	return 0;
}

int ZMotionRobot::switch_enable(bool enable) {
	int ret = 0;
	if (enable) {
		char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

		sprintf(cmdbuff, "RUNTASK 6, ROBOT_RESET");
		sprintf(tempbuff, "(%d)", robotId);
		strcat(cmdbuff, tempbuff);

		ret = ZController->sendCmd(cmdbuff, cmdbuffAck, 0);
	}
	else {
		emergency_stop();
	}

	return ret;
}

int ZMotionRobot::reset_line_num() {
	int stateIdxBase = get_state_idx_base();
	cmdNum = 0;
	ZController->set_axis_param(stateIdxBase + 3, "TABLE", 0);
	ZController->set_axis_param(stateIdxBase + 7, "TABLE", -1);

	// 将上一条轨迹类型置空，防止切换正逆解时判断轨迹未走完
	auto preTraj = trajectory.get_preTraj();
	TrajectoryPoint point = preTraj.get_point();
	point.trajType = TrajType::None;
	trajectory.set_preTraj(point);
	trajectory.set_previous_line_num(0);

	RBT_LOG_INFO(RobotLog::getLogger(), "My lgger: " << "R" << aliasId << " reset line num to 0.");
	//LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " reset line num to 0.");

	return 0;
}

int ZMotionRobot::set_jog_type(int type) {
	return 0;
}

int ZMotionRobot::jog_moving(int type, int idx, int dir, int move) {

	std::vector<std::vector<int>> axisIdx;
	std::vector<int> axis;
	// 关节轴
	axis = get_composed_axis({ get_joint_axis(), robotConfig.appAxisIdx });
	axisIdx.push_back(axis);
	// 世界坐标轴
	axis = get_composed_axis({ get_tcp_axis(), robotConfig.appAxisIdx });
	axisIdx.push_back(axis);
	// 工具坐标轴
	//axisIdx.push_back(axis);

	int ret = 0;
	if (type < 0 || type >= axisIdx.size() || idx < 0 || idx >= axisIdx[type].size()) {
		return -1;
	}

	// 未处于手动模式
	if (robotStatus.autoMode > 0) {
		robotStatus.upperStatus |= 0x10;
		return 1;
	}
	// 暂停状态下不可移动附加轴，允许停止
	if (get_bit(robotStatus.lowerStatus, 1) == 1 && idx > 5 && dir != 0) {
		robotStatus.upperStatus |= 0x04;
		return 2;
	}

	// 切换正逆解
	if (type < 1) {
		ret = switch_kinematics(1);
	}
	else {
		ret = switch_kinematics(-1);
	}
	// 正逆解切换失败
	if (ret != 0) {
		return -2;
	}

	// VMOVE 点动
	if (type < 2) {
		ZController->axis_jog(axisIdx[type][idx], dir);
	}
	// MOVE 点动
	else {
	}

	return ret;
}

int ZMotionRobot::save_task_status(bool enable, int inBuffer) {
	int stateIdxBase = get_state_idx_base();
	std::vector<int> axis = get_execute_axis();
	int ret = 0;

	if (inBuffer < 0) {
		ret = ZController->set_axis_param(stateIdxBase + 100, "TABLE", enable);
	}
	else {
		ret = ZController->set_axis_param(stateIdxBase + 100, "TABLE", enable, axis[0]);
	}

	return 0;
}


int ZMotionRobot::task_pause() {
	int stateIdxBase = get_state_idx_base();
	ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 2 });

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task pause.");
	return 0;
}

int ZMotionRobot::task_resume() {
	// 不在自动模式
	if (robotStatus.autoMode <= 0)
		return 1;

	// 获取保存状态
	RobotStatus savedState;
	read_saved_status(savedState);
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " saved mode: " << savedState.fkMode << ", cur mode: " << robotStatus.fkMode);

	// 切换到暂停前的状态
	for (size_t i = 0; i < 10; ++i) {
		if (savedState.fkMode != robotStatus.fkMode) {
			// 切换正逆解
			switch_kinematics(savedState.fkMode);

			// 等待切换
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " kinematics mode changed: " << robotStatus.fkMode);
			break;
		}

		if (i == 9) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " resume failed, cannot change kinematics.");
			return -1;
		}
	}

	// 暂停后是否运动
	float dist = 0.0;
	for (size_t i = 0; i < 9; ++i) {
		dist += (robotStatus.jPos[i] - savedState.jPos[i]) * (robotStatus.jPos[i] - savedState.jPos[i]);
	}
	dist = std::sqrt(dist);
	

	if (dist > 1e-2) {
		// 回到起点
		std::vector<int> axis = get_tcp_axis();
		int ret = moveLABS(axis, robotStatus.cPos, savedState.cPos, {});
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " "
			<< "from: " << vector_to_string(robotStatus.cPos) << "\n"
			<< "move to: " << vector_to_string(savedState.cPos)
		);

		// 等待运动完成
		while (true) {
			// 异常退出
			if (((robotStatus.lowerStatus >> 2) != 0) || (robotStatus.upperStatus > 0)) {
				return -3;
			}

			// IDLE 标志位
			std::vector<float> value;
			ZController->get_axis_param({ get_joint_axis()[0], get_tcp_axis()[0] }, "IDLE", value);

			if (std::fabs(value[0] + value[1] + 1) < 1e-2) {
				LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " reach desired point.");
				break;
			}
		}
	}
	

	int stateIdxBase = get_state_idx_base();
	ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 1 });

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task resume.");
	return 0;
}

int ZMotionRobot::task_stop() {
	// 轨迹清空
	trajectory.clear();

	// 停止记录位置
	save_task_status(false, -1);

	// 清空执行轴，摆焊轴
	std::vector<int> axis;
	auto camAxis = get_execute_axis();
	axis.push_back(camAxis[0]);
	//camAxis = get_tcp_axis();
	//axis.push_back(camAxis[0]);
	camAxis = get_cam_axis();
	axis.insert(axis.end(), camAxis.begin(), camAxis.end());

	// 轴停止，清空已下发任务
	ZController->axis_stop(axis);
	// 清除上位机异常码
	reset_upperStatus(-1);

	// 摆焊轴位置回零
	auto zeroPos = std::vector<float>(camAxis.size(), 0);
	ZController->set_axis_param(camAxis, "DPOS", zeroPos);

	// 轨迹序号复位
	reset_line_num();

	// 下位机复位
	ZController->set_axis_param(get_state_idx_base(), "TABLE", 0);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task stop.");

	return 0;
}

int ZMotionRobot::emergency_stop() {

	int stateIdxBase = get_state_idx_base();
	trajectory.clear();
	ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 3 });

	// 上位机下发停止
	//set_upperStatus(0x08);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " emergency stop.");
	return 0;

}

// 设备操作
int ZMotionRobot::device_operation() {
	return 0;
}



/* *************************** 自定义成员函数 *************************** */
ZMotionRobot::ZMotionRobot() {
}

ZMotionRobot::~ZMotionRobot() {
}

std::vector<int> ZMotionRobot::get_cam_axis() {
	int base = robotId * 32;
	std::vector<int> axis = { base + 27,base + 28,base + 29 };
	return axis;
}

std::vector<int> ZMotionRobot::get_tcp_axis() {
	int base = robotId * 32;
	std::vector<int> axis = { base + 15,base + 16,base + 17,base + 12,base + 13,base + 14 };
	return axis;
}

std::vector<int> ZMotionRobot::get_robot_tcp_axis() {
	int base = robotId * 32;
	std::vector<int> axis = { base + 9,base + 10,base + 11,base + 12,base + 13,base + 14 };
	return axis;
}



// 修改摆焊凸轮表
int ZMotionRobot::get_swing_num() {
	auto curTraj = trajectory.get_curTraj();
	Weave waveCfg = deserialize_Weave(curTraj.get_appendix());
	return std::round(trajectory.get_dist() / (curTraj.get_speed() / waveCfg.Freq));
}

int ZMotionRobot::update_swing_table(const Weave& waveCfg) {

	// 摆焊未启用
	if (waveCfg.Id <= 0) {
		return 1;
	}

	// 运动轴号
	std::vector<int> axis = get_execute_axis();

	// 凸轮表起始索引
	size_t sinTableBeg = 7000 + 2000 * robotId + 1000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;
	int ret = 0;
	// 检测是否需要修改凸轮表
	bool resetTable = true;
	std::vector<float> waveGenerator(numInterp, 0);

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.RightWidth;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;

	float detAmpl = (waveCfg.LeftWidth - waveCfg.RightWidth) / (waveCfg.LeftWidth + waveCfg.RightWidth);
	float detQ = std::asin(detAmpl);
	// 凸轮表连续：机器人停止 | 停留时间为0
	if (holdType > 0 || waveCfg.Dwell_left + waveCfg.Dwell_right < 1e-3) {
		// 左右摆幅不同
		if (std::fabs(waveCfg.LeftWidth - waveCfg.RightWidth) > 1e-1) {
			for (size_t i = 0; i < numInterp; ++i) {
				waveGenerator[i] = std::sin(2 * DT_PI * i / (numInterp - 1) + detQ) - detAmpl;
				waveGenerator[i] /= (1 - detAmpl);
			}

			// 缓冲中写入凸轮表
			for (size_t i = 0; i < numInterp; ++i) {
				ZController->set_axis_param(sinTableBeg + i, "TABLE", waveGenerator[i], axis[0]);
			}
		}
	}
	// 摆动停止: 判断条件与swing_on中对齐
	else if (holdType == 0 && waveCfg.Dwell_left + waveCfg.Dwell_right > 1e-3) {
		swingHoldTime = waveCfg.Dwell_left + waveCfg.Dwell_right;
		// 周期时间(ms)
		float totalTime = 1000 / freq + swingHoldTime;
		// 四分之一摆动周期占用的 table 个数
		size_t numQuarter = numInterp * (1000 / freq) / totalTime / 4;
		// 右停留时间占用的 table 个数
		size_t numRightHold = (numInterp - 4 * numQuarter) * waveCfg.Dwell_right / swingHoldTime;
		// 
		int numOffset = numQuarter * std::asin(detAmpl) * 2 / DT_PI;

		// 构造凸轮表
		size_t begIdx = 0, endIdx = numQuarter - numOffset;
		for (size_t i = begIdx; i < endIdx; ++i) {
			waveGenerator[i] = std::sin(2 * DT_PI * i / (4 * numQuarter - 1) + detQ) - detAmpl;
			waveGenerator[i] /= (1 - detAmpl);
		}
		begIdx = endIdx;
		endIdx += numRightHold;
		for (size_t i = begIdx; i < endIdx; ++i) {
			waveGenerator[i] = 1;
		}
		begIdx = endIdx;
		endIdx += numQuarter * 2;
		for (size_t i = begIdx; i < endIdx; ++i) {
			waveGenerator[i] = std::sin(2 * DT_PI * (i - numRightHold) / (4 * numQuarter - 1) + detQ) - detAmpl;
			waveGenerator[i] /= (1 - detAmpl);
		}
		begIdx = endIdx;
		endIdx += numInterp - 4 * numQuarter - numRightHold;
		for (size_t i = begIdx; i < endIdx; ++i) {
			waveGenerator[i] = (-1 - detAmpl) / (1 - detAmpl);
		}
		begIdx = endIdx;
		endIdx = numInterp;
		for (size_t i = begIdx; i < endIdx; ++i) {
			waveGenerator[i] = std::sin(2 * DT_PI * (i - numInterp + 4 * numQuarter) / (4 * numQuarter - 1) + detQ) - detAmpl;
			waveGenerator[i] /= (1 - detAmpl);
		}

		// 缓冲中写入凸轮表
		for (size_t i = 0; i < numInterp; ++i) {
			ZController->set_axis_param(sinTableBeg + i, "TABLE", waveGenerator[i], axis[0]);
		}
	}

	return ret;
}


int ZMotionRobot::swing_on(float dist, const Weave& waveCfg, int mode, const std::vector<float>& toolDir, const std::vector<float>& planeDir) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	std::vector<int> axis = get_composed_axis({ get_execute_axis(), robotConfig.appAxisIdx });
	std::vector<int> camAxis = get_cam_axis();

	// 凸轮表起始索引
	size_t sinTableBeg = 7000 + 2000 * robotId + 1000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	//float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	float ampl = waveCfg.RightWidth;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	if (holdType == 0) {
		swingHoldTime = waveCfg.Dwell_left + waveCfg.Dwell_right;
	}

	bool sinTableFlag = ((holdType > 0 || waveCfg.Dwell_left + waveCfg.Dwell_right < 1e-3)  \
		&& std::fabs(waveCfg.LeftWidth - waveCfg.RightWidth) > 1e-1)                        \
		|| (holdType == 0 && waveCfg.Dwell_left + waveCfg.Dwell_right > 1e-3);
	// 使用全局凸轮表
	if (!sinTableFlag)
		sinTableBeg = 6000;

	// 周期长度
	//float dist = vel * (1 / freq + swingHoldTime / 1000);

	Eigen::Vector3f zDir(0, 0, 0);
	// 自动计算焊枪角度
	if (toolDir.size() < 3) {
		//// 读取缓冲最终位置处的欧拉角(deg)
		//Eigen::Vector3f zEuler(0, 0, 0);
		//for (size_t i = 0; i < 3; ++i) {
		//	//ret = ZAux_Direct_GetEndMoveBuffer(handle_, tcpAngleAxisIdx[i], &zEuler[i]);
		//	ZController->get_axis_param(tcpAngleAxisIdx[i], "ENDMOVE", zEuler[i]);
		//}
		//zEuler *= DT_PI / 180;
		//zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
		//zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
		//zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	}
	// 给定焊枪角度
	else {
		zDir = Eigen::Vector3f(toolDir[0], toolDir[1], toolDir[2]);
	}

	// 设置摆角
	//if (toolDir.size() > 0) {
	//	Eigen::Vector3f tanDir(toolDir[0], toolDir[1], toolDir[2]);
	//	tanDir.normalize();
	//	zDir = (Eigen::AngleAxisf(waveCfg.Angle_Ltype_top * M_PI / 180, tanDir) * zDir).eval();
	//}

	float vectorBuffered2 = 0.0;
	ZController->get_axis_param(axis[0], "VECTOR_BUFFERED2", vectorBuffered2);

	//生成命令
	if (mode == 5) {
		sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f,%f,%f,%f)",
			camAxis[0], camAxis[1], camAxis[2],
			// mode, 主轴, 矢量距离, 周期长度, 左右摆幅, 开始Table, 结束Table
			mode, axis[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1,
			zDir[0], zDir[1], zDir[2],
			planeDir[0], planeDir[1], planeDir[2]
		);
	}
	else {
		sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
			camAxis[0], camAxis[1], camAxis[2],
			// mode, 主轴, 矢量距离, 周期长度, 左右摆幅, 开始Table, 结束Table
			mode, axis[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1,
			zDir[0], zDir[1], zDir[2]
		);
	}
	//std::cout << cmdbuff  << std::endl;

	//调用命令执行函数
	ZController->sendCmd(cmdbuff, cmdbuffAck);

	//LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Update swing config: " <<
	//	vector_to_string(serialize_Weave(waveCfg).second, 2)
	//);

	return ret;
}


int ZMotionRobot::swing_off(float displacement) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	std::vector<int> axis = get_execute_axis();
	std::vector<int> camAxis = get_cam_axis();

	float vectorBuffered2 = 0.0;
	ZController->get_axis_param(axis[0], "VECTOR_BUFFERED2", vectorBuffered2);
	//vectorBuffered2 += displacement;

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f)",
		camAxis[0], camAxis[1], camAxis[2],
		// mode, 主轴, 矢量距离
		-1, axis[0], vectorBuffered2
	);
	//std::cout << cmdbuff << std::endl;

	//调用命令执行函数
	ZController->sendCmd(cmdbuff, cmdbuffAck);

	//LOG_INFO("%s    ' Return: %d", cmdbuff, ret);

	return ret;
}

int ZMotionRobot::switch_kinematics(int mode, int retry) {
	int stateIdxBase = get_state_idx_base();
	int ret = 0, curFkMode = stateIdxBase + 2, fkCmd = stateIdxBase + 51;
	float readVal;

	for (size_t i = 0; i < retry + 1; ++i) {

		// 当前正逆解状态
		ret = ZController->get_axis_param(curFkMode, "TABLE", readVal);

		// 判断是否切换完成
		if (std::fabs(readVal - mode) < 0.1) {
			return ret;
		}

		// 切换一次正逆解
		ret = ZController->set_axis_param(fkCmd, "TABLE", mode);

	}

	return -1;
}

bool ZMotionRobot::kinematics_mached() {

	// 无运动缓冲
	if (trajectory.trajectory_loaded()) {
		return false;
	}

	auto curTraj = trajectory.get_curTraj();

	// 正逆解已匹配
	if ((robotStatus.fkMode > 0 && curTraj.isJoint()) ||                               // 正解模式
		(robotStatus.fkMode == -2 && !curTraj.isJoint() && curTraj.isBaseMotion()) ||  // 机器人坐标系
		(robotStatus.fkMode == -1 && !curTraj.isJoint() && !curTraj.isBaseMotion())    // 世界坐标系
		) {
		return true;
	}
	else {
		return false;
	}
}


int ZMotionRobot::read_saved_status(RobotStatus& status) {

	int cfgIdxBase = get_config_idx_base();
	std::vector<float> data;
	std::vector<int> axis(24, cfgIdxBase + 550);
	for (size_t i = 0; i < axis.size(); ++i) {
		axis[i] += i;
	}

	ZController->get_axis_param(axis, "VR", data);

	// 保存数据功能未使能或异常
	if (data[0] != 1) {
		return -1;
	}

	status.fkMode = static_cast<int>(data[1]);

	status.cmdNum = static_cast<int>(data[2]);

	status.jPos = std::vector<float>(data.begin() + 3, data.begin() + 9);
	status.jPos.insert(status.jPos.end(), data.begin() + 15, data.begin() + 18);

	status.cPosRaw = std::vector<float>(data.begin() + 9, data.begin() + 15);
	status.cPosRaw.insert(status.cPosRaw.end(), data.begin() + 15, data.begin() + 18);

	status.posOffset = std::vector<float>(data.begin() + 18, data.begin() + 24);

	// 局部坐标系转世界坐标系
	status.cPos = status.cPosRaw;
	cpos_base_to_world(status.cPos);

	return 0;
}

int ZMotionRobot::get_local_world_dpos(std::vector<float>& dpos) {
	std::vector<int> axis;
	axis = get_composed_axis({ get_tcp_axis(), robotConfig.appAxisIdxRead });
	return ZController->get_axis_param(axis, "WORLD_DPOS", dpos);
}

int ZMotionRobot::cpos_base_to_world(std::vector<float>& cPos) {
	auto rotMat = robotConfig.get_slave_calibratino_mat();

	auto euler = std::vector<float>(cPos.begin() + 3, cPos.begin() + 6);
	Eigen::Matrix3f curMat = Eigen::AngleAxisf(euler[2] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(euler[1] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(euler[0] * DT_PI / 180, Eigen::Vector3f::UnitX()).matrix();
	auto afterEuler = (rotMat * curMat).eulerAngles(2, 1, 0);

	for (size_t i = 0; i < 3; ++i) {
		cPos[3 + i] = afterEuler[2 - i] * 180 / DT_PI;
	}

	return 0;
}


TrajectoryPoint ZMotionRobot::partition_trajectory(const TrajectoryPoint& preTraj, const TrajectoryPoint& curTraj, DT_scale begRatio, DT_scale endRatio, int mode) {

	// 获取节点目标位置
	auto curPoint = curTraj.mainPoint;
	auto prePoint = preTraj.mainPoint;
	auto midPoint = curTraj.auxPoint;

	int num = curPoint.size();
	// 分段结果
	TrajectoryPoint ans(num);
	ans.trajType = curTraj.trajType;
	std::vector<DT_scale> relEndMove(num, 0);

	// 附加轴相对变化量
	for (size_t i = 0; i < num; ++i)
		relEndMove[i] = curPoint[i] - prePoint[i];

	// 欧拉角相对变化量
	if (curPoint.size() > 5) {
		auto begEuler = Eigen::Matrix<DT_scale, 3, 1>(prePoint[3], prePoint[4], prePoint[5]);
		auto midEuler = Eigen::Matrix<DT_scale, 3, 1>(midPoint[3], midPoint[4], midPoint[5]);
		auto endEuler = Eigen::Matrix<DT_scale, 3, 1>(curPoint[3], curPoint[4], curPoint[5]);
		auto relEuler = get_zyx_euler_distance(begEuler, midEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
	}

	bool isArc = (curTraj.trajType == TrajType::Arc);

	// 计算位置分量
	auto trajInfo = calc_traj_info(prePoint, midPoint, curPoint, isArc);
	DT_scale partial = 0;
	// 圆弧运动
	if (isArc) {
		// Eigen 类型的点位，用于计算
		Eigen::Vector3f rotNorm(trajInfo[4], trajInfo[5], trajInfo[6]), centerPos(trajInfo[0], trajInfo[1], trajInfo[2]);

		// 轨迹总旋转角度
		DT_scale theta = rotNorm.norm();
		rotNorm.normalize();
		// 起点处的半径
		Eigen::Vector3f radiusDir(0, 0, 0);
		for (size_t i = 0; i < 3; ++i) {
			radiusDir[i] = prePoint[i] - centerPos[i];
		}
		// 分段点位置
		Eigen::Vector3f arcPos;

		// 中间点处的比例
		partial = (mode == 0) ? (begRatio + endRatio) / 2 : (begRatio + endRatio) / 2 / (theta * radiusDir.norm());
		arcPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
		// 位置分量单独计算，姿态和附加值按线性累加
		for (size_t i = 0; i < num; ++i) {
			ans.auxPoint[i] = i < 3 ? arcPos[i] : (prePoint[i] + relEndMove[i] * partial);
		}

		// 终点处的比例
		partial = (mode == 0) ? endRatio : endRatio / (theta * radiusDir.norm());
		arcPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
		// 位置分量单独计算，姿态和附加值按线性累加
		for (size_t i = 0; i < num; ++i) {
			ans.mainPoint[i] = i < 3 ? arcPos[i] : (prePoint[i] + relEndMove[i] * partial);
		}
	}
	// 直线运动
	else {
		// 比例
		partial = (mode == 0) ? (begRatio + endRatio) / 2 : (begRatio + endRatio) / 2 / trajInfo[3];
		//if (partial > 1)
		//	partial = 1;
		for (size_t i = 0; i < num; ++i)
			ans.auxPoint[i] = prePoint[i] + relEndMove[i] * partial;

		// 比例
		partial = (mode == 0) ? endRatio : endRatio / trajInfo[3];
		//if (partial > 1)
		//	partial = 1;
		for (size_t i = 0; i < num; ++i)
			ans.mainPoint[i] = prePoint[i] + relEndMove[i] * partial;
	}

	//if (std::fabs(endRatio-1) < 1e-2) {
	//	ans.mainPoint = curPoint;
	//}

	return ans;
}


} // namespace FSAIRobotInterface