
#include "robot_interface/ZRVRobot.h"

#include "RobotLogger.h"

namespace FSAIRobotInterface {

	//! 获取下发指令轴号，主要用于确定运动主轴
	std::vector<int> ZRVRobot::get_execute_axis() {
		int base = robotId * 32;
		std::vector<int> axis = { base + 15,base + 16,base + 17, base + 12, base + 13, base + 14 };
		return axis;
	}

	// 机器人状态
	int ZRVRobot::update_rt_robot_status() {

		int stateIdxBase = get_state_idx_base();
		RobotStatus tmp;
		// !减少读取次数，优化读取速度
		std::vector<float> value;
		std::vector<int> idx(50, stateIdxBase);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}

		ZController->get_axis_param(idx, "TABLE", value);
		// 运动状态
		tmp.lowerStatus = static_cast<int>(value[0]);
		// 手动/自动模式
		tmp.autoMode = static_cast<int>(value[1]);
		// 正逆解模式
		tmp.fkMode = static_cast<int>(value[2]);
		// 运动行号
		tmp.lineNum = static_cast<int>(value[3]);
		// 轨迹编号
		//tmp.cmdNum = static_cast<int>(value[7]);
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

		// 焊接总时长
		tmp.weldTime = static_cast<long>(value[30]);
		// 起弧时间
		tmp.weldBegTime = static_cast<long>(value[31]);
		// 息弧时间
		tmp.weldEndTime = static_cast<long>(value[32]);

		// 局部坐标系转世界坐标系
		tmp.cPos = tmp.cPosRaw;
		cpos_base_to_world(tmp.cPos);

		// 缓冲中目标位置
		tmp.cPosBuffer = std::vector<float>(value.begin() + 40, value.begin() + 49);

		// 加锁
		std::lock_guard<std::mutex> lock(mtx);
		// 需要保持的状态
		tmp.upperStatus = robotStatus.upperStatus;

		robotStatus = tmp;

		return 0;
	}

	int ZRVRobot::get_all_robot_status(RobotStatus& status) {
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
		axis = get_composed_axis({ get_joint_axis(), robotConfig.appAxisIdxRead });
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


	int ZRVRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
		return 0;
	}
	int ZRVRobot::moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		return ZController->baseCMD(axis, "MOVERV_JABS", end);
	}

	int ZRVRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {

		std::vector<float> cmdData = { -1.0 };
		cmdData.insert(cmdData.end(), relMove.begin(), relMove.end());

		return ZController->baseCMD(axis, "MOVERV_LABS", cmdData);

	}
	int ZRVRobot::moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		
		std::vector<float> cmdData = { -1.0 };
		cmdData.insert(cmdData.end(), end.begin(), end.end());

		return ZController->baseCMD(axis, "MOVERV_LABS", cmdData);

	}

	int ZRVRobot::moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		return 0;
	}
	int ZRVRobot::moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {

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

	int ZRVRobot::set_manual_speed(float ratio) {

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
	int ZRVRobot::update_swing_config() {

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
		// 修改摆动参数
		int numPeriod = get_swing_num();
		if (waveCfg.Id > 0 && numPeriod > 0 && waveCfg.Shape == 0) {
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

			int swingMode = curTraj.isLine() ? 3 : 5;
			ret = swing_on(trajectory.get_dist() / numPeriod, waveCfg, swingMode, zDir, nDir);

			// 计算轴运动距离
			ret += swing_off(trajectory.get_dist());
		}

		return 0;
	}

	int ZRVRobot::update_track_config() {

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

	int ZRVRobot::update_welder_config() {

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
		if (weldCfg.WeldingWorkMode == 4) {
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

	int ZRVRobot::get_remain_buffer() {
		int idx = get_state_idx_base() + 5;
		float value;

		ZController->get_axis_param(idx, "TABLE", value);

		return static_cast<int>(value);
	}

	int ZRVRobot::push_new_trajectory(DiscreteTrajectory trajList) {

		// 未设置自动模式
		//if (robotStatus.autoMode <= 0) {
		//	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " switch to auto mode before push trajectory.");
		//	set_upperStatus(0x10);
		//	return -1;
		//}

		// 轨迹为空
		if (trajList.size() == 0) {
			return -2;
		}

		// 轨迹预处理
		// 按主从标定矩阵，将世界坐标系姿态转换为基坐标系姿态
		//auto rotMat = robotConfig.get_slave_calibratino_mat().inverse();
		//trajList.apply_rotate(rotMat);

		std::unique_lock<std::mutex> lock(mtx);
		// 等待条件置反
		motionDone = false;

		// 轨迹入栈
		trajectory.push_new_trajectory(trajList);
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " receive new trajectory, trajectory buffer size is " << trajectory.size());

		return 0;

	}


	int ZRVRobot::execute_single_joint() {
		int ret = 0;
		// 前一条轨迹
		auto preTraj = trajectory.get_preTraj();
		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();

		std::vector<int> axis = get_composed_axis({ get_joint_axis(), robotConfig.appAxisIdx });

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
		ZController->set_axis_param(axis[0], (char*)"FORCE_SPEED", curTraj.get_speed() / 100.0);
		// 设置平滑度
		if (curTraj.get_smooth() > 0) {
			ZController->set_axis_param(axis[0], (char*)"ZSMOOTH", curTraj.get_smooth());
		}

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveJABS: " << vector_to_string(pnt));
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
			<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : ""));

		// 开始记录位置
		//save_task_status(true);
		// 下发轨迹编号
		//send_running_line_num(axis[0], traj.get_curTraj());

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
		//save_task_status(false);

		// 轨迹出栈
		if (ret == 0) {
			trajectory.next();
		}
		else {
			return -1;
		}

		return 0;
	}

	int ZRVRobot::execute_single_cartesian() {

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
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : "")
			<< ". traj dist: " << trajectory.get_dist()
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
		ZController->set_axis_param(axis[0], "FORCE_SPEED", curTraj.get_speed());

		// 开始记录位置
		//save_task_status(true);
		// 下发轨迹编号
		//send_running_line_num(axis[0], traj.get_curTraj());


		// 当前轨迹的相对运动量
		std::vector<float> relEndMove = trajectory.get_relative_distance();

		// 开启摆焊
		int numPeriod = get_swing_num();
		if (waveCfg.Id > 0) {
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
					if (i > 0 && i < numPeriod - 1)
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
					if (i > 0 && i < numPeriod - 1)
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
				ZController->set_base_param(axis[0], "MOVE_WA", { 1.0 });
			}
		}
		// 无摆焊，正常下发
		else {
			if (curTraj.isArc()) {
				moveCABS(axis, prePoint, midPoint, curPoint, 0, mask);
			}
			else if (curTraj.isLine()) {
				moveLABS(axis, prePoint, curPoint, mask);
			}
		}

		// 更新轨迹编号
		trajectory.trajList.front().lineNum = ++cmdNum;

		// 下发轨迹序号
		send_line_num(axis[0], trajectory.get_curTraj());
		// 停止记录位置
		//save_task_status(false);


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


	int ZRVRobot::send_line_num(int axis, const SingleTrajectory &curTraj) {
		int stateIdxBase = get_state_idx_base();
		int ret = 0;

		axis = get_execute_axis()[0];
		ret = ZController->set_axis_param(stateIdxBase + 3, "TABLE", curTraj.lineNum, axis);

		LOG4CPLUS_INFO(RobotLog::getLogger(),
			"R" << aliasId << " send point with line num: " << curTraj.lineNum
		);

		return ret;
	}

	// 剩余缓冲检测
	int ZRVRobot::remain_buffer_free() {
		// 获取缓存长度
		int remainBuffer = get_remain_buffer();

		return remainBuffer > 1000;
	}

	// 一致性轨迹预处理，可以连续下发的轨迹
	int ZRVRobot::set_ready_for_consistent_traj() {

		if (trajectory.trajectory_loaded())
			return 0;

		auto curTraj = trajectory.get_curTraj();
		auto preTraj = trajectory.get_preTraj();
		int ret = 0;

		// 设定空间运动起点
		if (preTraj.trajType == TrajType::None || (preTraj.isJoint() && !curTraj.isJoint())) {

			TrajectoryPoint point;
			point.trajType = TrajType::Line;
			point.mainPoint = robotStatus.cPosBuffer;
			point.auxPoint = robotStatus.cPosBuffer;
			trajectory.set_preTraj(point);

			auto printPoint = curTraj.mainPoint;
			cpos_base_to_world(printPoint);
			LOG4CPLUS_INFO(RobotLog::getLogger(),
				"R" << aliasId << " trajectory part begin "
				<< "CurPos: " << vector_to_string(point.mainPoint)
			);

		}

		return 1;
	}

	// 一致性轨迹就绪
	int ZRVRobot::consistent_traj_ready(int& state) {

		auto curTraj = trajectory.get_curTraj();
		auto preTraj = trajectory.get_preTraj();
		int ret = 0;

		// 正逆解变化
		if (preTraj.isJoint() && !curTraj.isJoint()) {
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

	int ZRVRobot::separate_trajectory() {
		auto ite = trajectory.trajList.begin();

		// 关节轨迹无需分段
		if (ite->isJoint()) {
			return 1;
		}

		// 计算轨迹参数
		trajectory.calc_traj_info();

		return 0;
	}

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief 读取VR寄存器中的配置参数
	*/
	int ZRVRobot::read_register_config() {
		int cfgIdxBase = get_config_idx_base();
		int ret = 0;
		std::vector<int> configIdx;
		std::vector<float> readValue;

		// 连杆长度
		configIdx = std::vector<int>(12, cfgIdxBase + 2);
		for (size_t i = 0; i < configIdx.size(); ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", readValue);
		robotConfig.linkLength = std::vector<float>(configIdx.size(), 0);
		for (size_t i = 0; i < configIdx.size(); ++i) {
			robotConfig.linkLength[i] = readValue[i];
		}

		// 附加轴编号
		configIdx = { cfgIdxBase + 205, cfgIdxBase + 206, cfgIdxBase + 207, cfgIdxBase + 208, cfgIdxBase + 209, cfgIdxBase + 210 };
		ret = ZController->get_axis_param(configIdx, "VR", readValue);
		robotConfig.appAxisIdx.resize(3);
		robotConfig.appAxisIdxRead.resize(3);
		for (size_t i = 0; i < robotConfig.appAxisIdx.size(); ++i) {
			// 下发
			robotConfig.appAxisIdx[i] = static_cast<int>(readValue[i]);
			// 读取
			robotConfig.appAxisIdxRead[i] = static_cast<int>(readValue[i + 3]);
		}

		// 编码器位数
		configIdx = std::vector<int>(9, cfgIdxBase + 20);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.encoderBit);

		// 传动比
		//configIdx = std::vector<int>(9, cfgIdxBase + 30);
		//for (size_t i = 0; i < 9; ++i) {
		//	configIdx[i] += i;
		//}
		//ret = ZController->get_axis_param(configIdx, "VR", robotConfig.transRatio);
		// 传动比分子
		configIdx = std::vector<int>(9, cfgIdxBase + 140);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.transRatioNumerator);
		// 传动比分母
		configIdx = std::vector<int>(9, cfgIdxBase + 150);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.transRatioDenominator);

		// TCP
		configIdx = std::vector<int>(6, cfgIdxBase + 111);
		for (size_t i = 0; i < 6; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.tcpPose);

		// 关节上限位
		configIdx = std::vector<int>(9, cfgIdxBase + 50);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.jointSupremum);

		// 关节下限位
		configIdx = std::vector<int>(9, cfgIdxBase + 40);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.jointInfimum);

		// 最大关节速度(自动)
		configIdx = std::vector<int>(9, cfgIdxBase + 60);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.maxJointSpeedAuto);

		// 最大关节速度(手动)
		configIdx = std::vector<int>(9, cfgIdxBase + 70);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.maxJointSpeedManual);

		// 最大末端速度(手动)
		configIdx = { cfgIdxBase + 85, cfgIdxBase + 86 };
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.maxCartSpeedManual);

		// IO 配置

		// 附加轴标定结果
		configIdx = std::vector<int>(9, cfgIdxBase + 91);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.auxCalbration);

		// 零点编码器值
		configIdx = std::vector<int>(9, cfgIdxBase + 100);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.zeroEncoder);

		// 主从机标定结果
		configIdx = std::vector<int>(6, cfgIdxBase + 130);
		for (size_t i = 0; i < 6; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->get_axis_param(configIdx, "VR", robotConfig.slaveCalibration);

		return ret;
	}

	/**
	* @brief 更新VR寄存器中的配置参数
	*/
	int ZRVRobot::write_register_config(const RobotConfig& config) {

		int cfgIdxBase = get_config_idx_base();
		int ret = 0;
		std::vector<int> configIdx;
		std::vector<float> readValue;

		configIdx = std::vector<int>(9, cfgIdxBase + 20);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", robotConfig.encoderBit);

		// 传动比
		//configIdx = std::vector<int>(9, cfgIdxBase + 30);
		//for (size_t i = 0; i < 9; ++i) {
		//	configIdx[i] += i;
		//}
		//ret = ZController->set_axis_param(configIdx, "VR", config.transRatio);
		// 传动比分子
		configIdx = std::vector<int>(9, cfgIdxBase + 140);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.transRatioNumerator);
		// 传动比分母
		configIdx = std::vector<int>(9, cfgIdxBase + 150);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.transRatioDenominator);

		// TCP
		configIdx = std::vector<int>(6, cfgIdxBase + 111);
		for (size_t i = 0; i < 6; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.tcpPose);

		// 关节上限位
		configIdx = std::vector<int>(9, cfgIdxBase + 50);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.jointSupremum);

		// 关节下限位
		configIdx = std::vector<int>(9, cfgIdxBase + 40);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.jointInfimum);

		// 最大关节速度(自动)
		configIdx = std::vector<int>(9, cfgIdxBase + 60);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.maxJointSpeedAuto);

		// IO 配置

		// 附加轴标定结果
		configIdx = std::vector<int>(9, cfgIdxBase + 91);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.auxCalbration);

		// 零点编码器值
		configIdx = std::vector<int>(9, cfgIdxBase + 100);
		for (size_t i = 0; i < 9; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.zeroEncoder);

		// 主从机标定结果
		configIdx = std::vector<int>(6, cfgIdxBase + 130);
		for (size_t i = 0; i < 6; ++i) {
			configIdx[i] += i;
		}
		ret = ZController->set_axis_param(configIdx, "VR", config.slaveCalibration);

		// 写入控制卡后读取到本地
		read_register_config();

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " write register config.");

		return 0;
	}

	int ZRVRobot::switch_auto(bool enableAuto) {

		int stateIdxBase = get_state_idx_base();
		// 清除模式不匹配的异常
		robotStatus.upperStatus &= 0xEF;

		// 切换手动/自动模式
		ZController->set_axis_param(stateIdxBase + 50, "TABLE", enableAuto ? 1 : -1);

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
		point.trajType = TrajType::None;
		trajectory.set_preTraj(point);

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " switch to " << (enableAuto ? "auto" : "manual") << " mode."
			<< " upperStatus: " << robotStatus.upperStatus << ", "
			<< " TrajType: " << static_cast<int>(trajectory.get_preTraj().get_trajType()) << "\n"
			<< "CurJPos: " << vector_to_string(tmpStatus.jPos) << "\n"
			<< "CurCPos: " << vector_to_string(tmpStatus.cPos)
		);

		return 0;
	}

	int ZRVRobot::switch_enable(bool enable) {
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

	int ZRVRobot::reset_line_num() {
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

	int ZRVRobot::set_jog_type(int type) {
		return 0;
	}

	int ZRVRobot::jog_moving(int type, int idx, int dir, int move) {

		std::vector<std::vector<int>> axisIdx;
		std::vector<int> axis;
		// 关节轴
		axis = get_composed_axis({ get_joint_axis(), robotConfig.appAxisIdx });
		axisIdx.push_back(axis);
		// 世界坐标轴
		axis = get_composed_axis({ get_tcp_axis(), robotConfig.appAxisIdx });
		axisIdx.push_back(axis);

		int ret = 0;
		if (type < 0 || type >= axisIdx.size() || idx < 0 || idx >= axisIdx[type].size()) {
			return -1;
		}

		// 未处于手动模式
		if (robotStatus.autoMode > 0) {
			robotStatus.upperStatus |= 0x10;
			return 1;
		}
		// 暂停状态下不可移动附加轴
		if ((robotStatus.lowerStatus & 0x02) == 1 && idx > 5) {
			robotStatus.upperStatus |= 0x04;
			return 2;
		}

		// 关节点动
		if (type == 0) {
			if (dir == 0)
				ZController->axis_stop({ axisIdx[1][0] }, 4);
			else
				ZController->baseCMD({ axisIdx[type][idx] }, "MOVERV_J", { static_cast<float>(359.9 *  dir) });
		}
		// 世界坐标点动
		else if (type == 1) {
			ZController->axis_jog(axisIdx[type][idx], dir);
		}

		return ret;
	}

	int ZRVRobot::task_pause() {
		int stateIdxBase = get_state_idx_base();
		ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 2 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task pause.");
		return 0;
	}

	int ZRVRobot::task_resume() {
		// 不在自动模式
		if (robotStatus.autoMode <= 0) {
			return 1;
		}

		// 获取保存状态
		RobotStatus savedState;
		read_saved_status(savedState);
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " saved mode: " << savedState.fkMode << ", cur mode: " << robotStatus.fkMode);

		int stateIdxBase = get_state_idx_base();
		ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 1 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task resume.");
		return 0;
	}

	int ZRVRobot::task_stop() {
		// 轨迹清空
		trajectory.clear();

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
		set_upperStatus(0x08);

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

	int ZRVRobot::emergency_stop() {

		int stateIdxBase = get_state_idx_base();
		trajectory.clear();
		ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 3 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " emergency stop.");
		return 0;

	}

	// 设备操作
	int ZRVRobot::device_operation() {
		return 0;
	}


	/* *************************** 可修改接口 *************************** */

	/* *************************** 自定义成员函数 *************************** */
	ZRVRobot::ZRVRobot() {
	}

	ZRVRobot::~ZRVRobot() {
	}

	std::vector<int> ZRVRobot::get_cam_axis() {
		int base = robotId * 32;
		std::vector<int> axis = { base + 27,base + 28,base + 29 };
		return axis;
	}

	std::vector<int> ZRVRobot::get_joint_axis() {
		int base = robotId * 32;
		std::vector<int> axis = { base + 0,base + 1,base + 2,base + 3,base + 4,base + 5 };
		return axis;
	}

	std::vector<int> ZRVRobot::get_tcp_axis() {
		int base = robotId * 32;
		std::vector<int> axis = { base + 15,base + 16,base + 17,base + 12,base + 13,base + 14 };
		return axis;
	}

	std::vector<int> ZRVRobot::get_robot_tcp_axis() {
		int base = robotId * 32;
		std::vector<int> axis = { base + 9,base + 10,base + 11,base + 12,base + 13,base + 14 };
		return axis;
	}



	// 修改摆焊凸轮表
	int ZRVRobot::get_swing_num() {
		auto curTraj = trajectory.get_curTraj();
		Weave waveCfg = deserialize_Weave(curTraj.get_appendix());
		return std::round(trajectory.get_dist() / (curTraj.get_speed() / waveCfg.Freq));
	}

	int ZRVRobot::update_swing_table(const Weave& waveCfg) {

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


	int ZRVRobot::swing_on(float dist, const Weave& waveCfg, int mode, const std::vector<float>& toolDir, const std::vector<float>& planeDir) {
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

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Update swing config: " <<
			vector_to_string(serialize_Weave(waveCfg).second, 2)
		);

		return ret;
	}


	int ZRVRobot::swing_off(float displacement) {
		char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
		int ret = 0;

		std::vector<int> axis = get_execute_axis();
		std::vector<int> camAxis = get_cam_axis();

		float vectorBuffered2 = 0.0;
		ZController->get_axis_param(axis[0], "VECTOR_BUFFERED2", vectorBuffered2);
		vectorBuffered2 += displacement - 0.01;

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


	int ZRVRobot::read_saved_status(RobotStatus& status) {

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

	int ZRVRobot::get_local_world_dpos(std::vector<float>& dpos) {
		std::vector<int> axis;
		axis = get_composed_axis({ get_tcp_axis(), robotConfig.appAxisIdxRead });
		return ZController->get_axis_param(axis, "WORLD_DPOS", dpos);
	}

	int ZRVRobot::cpos_base_to_world(std::vector<float>& cPos) {
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

	TrajectoryPoint ZRVRobot::partition_trajectory(const TrajectoryPoint& preTraj, const TrajectoryPoint& curTraj, DT_scale begRatio, DT_scale endRatio, int mode) {

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

		// 四元数
		auto begQuat = Eigen::AngleAxisf(prePoint[5] * DT_PI / 180, Eigen::Vector3f::UnitZ())   \
			* Eigen::AngleAxisf(prePoint[4] * DT_PI / 180, Eigen::Vector3f::UnitY()) \
			* Eigen::AngleAxisf(prePoint[3] * DT_PI / 180, Eigen::Vector3f::UnitX());
		auto endQuat = Eigen::AngleAxisf(curPoint[5] * DT_PI / 180, Eigen::Vector3f::UnitZ())   \
			* Eigen::AngleAxisf(curPoint[4] * DT_PI / 180, Eigen::Vector3f::UnitY()) \
			* Eigen::AngleAxisf(curPoint[3] * DT_PI / 180, Eigen::Vector3f::UnitX());

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
			// 姿态分量
			auto euler = begQuat.slerp(partial, endQuat).matrix().eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				ans.auxPoint[3 + i] = euler[2 - i] * 180 / DT_PI;
			}

			// 终点处的比例
			partial = (mode == 0) ? endRatio : endRatio / (theta * radiusDir.norm());
			arcPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
			// 位置分量单独计算，姿态和附加值按线性累加
			for (size_t i = 0; i < num; ++i) {
				ans.mainPoint[i] = i < 3 ? arcPos[i] : (prePoint[i] + relEndMove[i] * partial);
			}
			// 姿态分量
			euler = begQuat.slerp(partial, endQuat).matrix().eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				ans.mainPoint[3 + i] = euler[2 - i] * 180 / DT_PI;
			}
		}
		// 直线运动
		else {
			// 比例
			partial = (mode == 0) ? (begRatio + endRatio) / 2 : (begRatio + endRatio) / 2 / trajInfo[3];
			// 位置分量
			for (size_t i = 0; i < num; ++i)
				ans.auxPoint[i] = prePoint[i] + relEndMove[i] * partial;
			// 姿态分量
			auto euler = begQuat.slerp(partial, endQuat).matrix().eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				ans.auxPoint[3 + i] = euler[2 - i] * 180 / DT_PI;
			}

			// 比例
			partial = (mode == 0) ? endRatio : endRatio / trajInfo[3];
			// 位置分量
			for (size_t i = 0; i < num; ++i)
				ans.mainPoint[i] = prePoint[i] + relEndMove[i] * partial;
			// 姿态分量
			euler = begQuat.slerp(partial, endQuat).matrix().eulerAngles(2, 1, 0);
			for (size_t i = 0; i < 3; ++i) {
				ans.mainPoint[3 + i] = euler[2 - i] * 180 / DT_PI;
			}
		}

		return ans;
	}


} // namespace FSAIRobotInterface