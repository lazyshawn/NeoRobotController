
#include "robot_interface/FSAIRobot.h"

#include "RobotLogger.h"

#ifndef CPPHTTPLIB_OPENSSL_SUPPORT
#define CPPHTTPLIB_OPENSSL_SUPPORT
#endif
#include <httplib.h>
#include <Windows.h>
#include <nlohmann/json.hpp>

#include <iostream>

namespace FSAIRobotInterface {

	//! Robot Web Services 客户端
	std::shared_ptr<httplib::Client> client;

	int RegisterBuffer::clear() {
		num = 0;
		buffer.clear();
		return 0;
	}

	bool RegisterBuffer::empty() {
		return buffer.empty();
	}

	int RegisterBuffer::add_buffer(int idx, float value) {
		buffer.push_back({ idx, value });
		return 0;
	}

	int RegisterBuffer::add_buffer(const std::vector<int>& idx, const std::vector<float>& value) {
		int num = (std::min)(idx.size(), value.size());
		for (size_t i = 0; i < num; ++i) {
			buffer.push_back({ idx[i], value[i] });
		}
		return 0;
	}

	int RegisterBuffer::pop(std::pair<int, float>& pair) {
		if (buffer.empty()) {
			return -1;
		}

		pair = buffer.front();
		buffer.pop_front();
		num++;

		return 0;
	}



	// 机器人管理类
	FSAIRobot::FSAIRobot() {
	}
	FSAIRobot::~FSAIRobot() {
		// 析构时先停止轴运动
		//task_stop();
		//emergency_stop();

		if (ZController) {
			ZController->remove_robot(robotId);
		}
	}


	std::vector<int> FSAIRobot::get_execute_axis() {
		int base = robotId * 32;
		//std::vector<int> axis = { base + 18,base + 19,base + 20,base + 21,base + 22,base + 23 };
		std::vector<int> axis = { base + 0,base + 1,base + 2,base + 3,base + 4,base + 5 };
		return axis;
	}

	
	/* *************************** 上层接口实现 *************************** */
	int FSAIRobot::cpos_base_to_world(std::vector<float>& cPos) {

		float tmp = cPos[3];
		cPos[3] = cPos[5];
		cPos[5] = tmp;

		return 0;
	}


	/* *************************** 底层接口实现 *************************** */
	int FSAIRobot::update_rt_robot_status() {

		int stateIdxBase = get_state_idx_base();
		RobotStatus tmp;
		// !减少读取次数，优化读取速度
		std::vector<float> value;
		std::vector<int> idx(9, 0);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}
		// 关节位置
		ZController->get_axis_param(idx, "DPOS", tmp.jPos);

		// 空间点位
		idx = std::vector<int>(9, 32);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}
		ZController->get_axis_param(idx, "DPOS", tmp.cPosRaw);
		// 地轨位置
		for (size_t i = 0; i < 3; ++i) {
			tmp.cPosRaw[6 + i] = tmp.jPos[6 + i];
		}

		tmp.cPos = tmp.cPosRaw;
		cpos_base_to_world(tmp.cPos);

		// 本体坐标系位置
		ZController->get_axis_param({ 50,51,52,53,54,55,6,7,8 }, "DPOS", tmp.cPosR);
		cpos_base_to_world(tmp.cPosR);

		//idx = std::vector<int>(50, get_state_idx_base());
		//for (size_t i = 0; i < idx.size(); ++i) {
		//	idx[i] += i;
		//}
		//ZController->get_axis_param(idx, "TABLE", value);
		ZController->get_register(stateIdxBase, 50, value, 0);

		// 机器人状态
		tmp.lowerStatus = static_cast<int>(value[0]);

		// 手动/自动模式
		tmp.autoMode = static_cast<int>(value[1]);

		// 轨迹编号
		tmp.lineNum = static_cast<int>(value[3]);

		// 当前时间戳
		tmp.slaveTime = static_cast<long>(value[28]);

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

		// 
		//ZController->get_axis_param({ stateIdxBase + 24117 }, "TABLE", value);
		//tmp.lineNum = static_cast<int>(value[0]);

		// 异常码
		ZController->get_register(stateIdxBase + 250, 20, value, 0);
		tmp.subErrorCode = std::vector<int>(value.size(), 0);
		for (size_t i = 0; i < value.size(); ++i) {
			tmp.subErrorCode[i] = static_cast<int>(value[i]);
		}

		// 加锁
		std::lock_guard<std::mutex> lock(mtxMotion);
		robotStatus = tmp;

		return 0;
	}

	int FSAIRobot::get_all_robot_status(RobotStatus& status) {

		{
			// 加锁
			std::lock_guard<std::mutex> lock(mtxMotion);

			// 更新机器人状态
			status = robotStatus;
		}

		// 轴号
		std::vector<int> axis;
		// 读取非实时参数
		std::vector<float> value;
		//// 机器人坐标系
		//axis = get_robot_tcp_axis();
		//ZController->get_axis_param(axis, "DPOS", status.cPosR);
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

		// 异常码辅码
		axis = std::vector<int>(20, get_state_idx_base());
		for (size_t i = 0; i < axis.size(); ++i) {
			axis[i] += i;
		}
		ZController->get_axis_param(axis, "TABLE", value);
		status.subErrorCode = std::vector<int>(value.size(), 0);
		for (size_t i = 0; i < axis.size(); ++i) {
			status.subErrorCode[i] = static_cast<int>(value[i]);
		}
		//ZController->get_axis_param(axis, "AXISSTATUS", value);
		//ZController->get_axis_param({ get_cmd_idx_base() + 24110 }, "TABLE", status.subErrorCode);

		return 0;
	}

	int FSAIRobot::get_local_world_dpos(std::vector<float>& dpos) {
		dpos = std::vector<float>(9, 0);
		return 0;
	}

	int FSAIRobot::get_remain_buffer() {
		int idx = get_state_idx_base() + 5;
		float value;

		ZController->get_axis_param(idx, "TABLE", value);

		return static_cast<int>(value);
	}

	int FSAIRobot::set_manual_speed(float ratio) {

		int stateIdxBase = get_cmd_idx_base() + 24111;
		if (robotStatus.autoMode > 0) {
			return 1;
		}

		std::vector<int> idx(5, stateIdxBase);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}

		// 保存到 table, 触发速度刷新
		ZController->set_axis_param(idx, "TABLE", { 1, static_cast<float>(200.0 * ratio / 100.0), 100, ratio, 100.0 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " set speed ratio: " << ratio / 100.0);

		return 0;
	}


	int FSAIRobot::switch_auto(bool enableAuto) {

		int stateIdxBase = get_state_idx_base();
		// 清除模式不匹配的异常
		robotStatus.upperStatus &= 0xEF;

		// 机器人运动中
		//if ((robotStatus.lowerStatus & 0x01) == 1) {
		//	return 1;
		//}

		// 已经处于指定模式
		//if ((robotStatus.autoMode > 0 && enableAuto) || (robotStatus.autoMode < 0 && !enableAuto)) {
		//	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " already in " << (enableAuto ? "auto" : "manual") << " mode");
		//	return 2;
		//}

		// 切换手动/自动模式
		int TableStartNum = get_cmd_idx_base();
		ZController->set_axis_param(stateIdxBase + 1, "TABLE", enableAuto ? 1 : -1);
		if (enableAuto) {
			ZController->set_axis_param(TableStartNum + 24000, "TABLE", 1);
			// 开始信号
			//ZController->set_axis_param(TableStartNum + 23996, "TABLE", 1);
		}
		else {
			ZController->set_axis_param(TableStartNum + 24001, "TABLE", 1);
			// 开始信号
			//ZController->set_axis_param(TableStartNum + 23996, "TABLE", 0);
		}

		RobotStatus tmpStatus;
		get_rt_robot_status(tmpStatus);

		// 检测是否切换成功

		// 设定轨迹起点
		TrajectoryPoint point;
		point.mainPoint = tmpStatus.jPos;
		point.trajType = TrajType::None;
		trajectory.set_preTraj(point);

		LOG4CPLUS_INFO(RobotLog::getLogger(),
			"R" << aliasId << " switch to " << (enableAuto ? "auto" : "manual") << " mode."
			<< " upperStatus: " << robotStatus.upperStatus << ", " 
			<< " TrajType: " << static_cast<int>(trajectory.get_preTraj().get_trajType()) << "\n"
			<< "CurJPos: " << vector_to_string(tmpStatus.jPos) << "\n"
			<< "CurCPos: " << vector_to_string(tmpStatus.cPos)
		);

		return 0;
	}


	int FSAIRobot::switch_enable(bool enable) {
		int ret = 0;
		if (enable) {
			char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

			sprintf(cmdbuff, "RUNTASK 10, ROBOT_RESET");
			sprintf(tempbuff, "(%d)", robotId);
			strcat(cmdbuff, tempbuff);

			ret = ZController->sendCmd(cmdbuff, cmdbuffAck, 0);
		}
		else {
			emergency_stop();
		}

		return ret;
	}


	int FSAIRobot::reset_line_num() {

		int stateIdxBase = get_cmd_idx_base();
		cmdNum = 0;
		//ZController->set_axis_param(stateIdxBase + 24123, "TABLE", 0);
		//// 下位机复位
		//ZController->set_axis_param(stateIdxBase + 23999, "TABLE", 1);
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//// 清空恢复
		//ZController->set_axis_param(stateIdxBase + 23995, "TABLE", 1);


		// 将上一条轨迹类型置空，防止切换正逆解时判断轨迹未走完
		auto preTraj = trajectory.get_preTraj();
		TrajectoryPoint point = preTraj.get_point();
		point.trajType = TrajType::None;
		trajectory.set_preTraj(point);
		trajectory.set_previous_line_num(0);

		RBT_LOG_INFO(RobotLog::getLogger(), "R" << aliasId << " reset line num to 0.");

		return 0;

	}


	int FSAIRobot::push_new_trajectory(DiscreteTrajectory trajList) {

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

		// 轨迹预处理: 欧拉角修正
		for (auto& traj : trajList.trajList) {
			if (traj.isJoint()) {}
			else {
				float tmp = traj.mainPoint[3];
				traj.mainPoint[3] = traj.mainPoint[5];
				traj.mainPoint[5] = tmp;

				tmp = traj.auxPoint[3];
				traj.auxPoint[3] = traj.auxPoint[5];
				traj.auxPoint[5] = tmp;
			}
		}

		std::unique_lock<std::mutex> lock(mtxMotion);
		// 等待条件置反
		motionDone = false;

		// 轨迹入栈
		trajectory.push_new_trajectory(trajList);
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " receive new trajectory, trajectory buffer size is " << trajectory.size());

		return 0;

	}


	int FSAIRobot::set_jog_type(int type) {
		if (type < 0) {
			return -1;
		}

		int idxBase = get_cmd_idx_base();

		// 关节
		if (type == 0) {
			ZController->set_axis_param(idxBase + 24003, "TABLE", 1);
		}
		// 世界坐标系
		else if (type == 1) {
			ZController->set_axis_param(idxBase + 24002, "TABLE", 1);
		}
		// 工具坐标系
		else if (type == 2) {
			ZController->set_axis_param(idxBase + 24005, "TABLE", 1);
		}
		// 基坐标系
		else if (type == 3) {
			ZController->set_axis_param(idxBase + 24004, "TABLE", 1);
		}

		return 0;
	}

	int FSAIRobot::jog_moving(int type, int idx, int dir, int move) {

		int ret = 0;
		if (type < 0 || idx < 0) {
			return -1;
		}

		// 未处于手动模式
		if (robotStatus.autoMode > 0) {
			robotStatus.upperStatus |= 0x10;
			return 1;
		}
		//// 暂停状态下不可移动附加轴
		//if ((robotStatus.lowerStatus & 0x02) == 1 && idx > 5) {
		//	robotStatus.upperStatus |= 0x04;
		//	return 2;
		//}

		int idxBase = get_cmd_idx_base();

		// 关节
		if (type == 0) {
			ZController->set_axis_param(idxBase + 24003, "TABLE", 1);
		}
		// 世界坐标系
		else if (type == 1) {
			ZController->set_axis_param(idxBase + 24002, "TABLE", 1);
		}
		// 工具坐标系
		else if (type == 2) {
			ZController->set_axis_param(idxBase + 24005, "TABLE", 1);
		}
		// 基坐标系
		else if (type == 3) {
			ZController->set_axis_param(idxBase + 24004, "TABLE", 1);
		}

		// 点动
		if (dir > 0) {
			ZController->set_axis_param(idxBase + 24010 + idx, "TABLE", 1);
		}
		else if (dir < 0) {
			ZController->set_axis_param(idxBase + 24030 + idx, "TABLE", 1);
		}
		// 停止
		else {
			ZController->set_axis_param(idxBase + 24010 + idx, "TABLE", -1);
			ZController->set_axis_param(idxBase + 24030 + idx, "TABLE", -1);
		}

		return ret;
	}

	int FSAIRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& moveCmd, const std::vector<int>& mask) {
		return 0;
	}
	int FSAIRobot::moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {

		int idx = get_point_idx_base();
		std::vector<int> idxList;
		std::vector<float> value;
		// 运动类型
		ZController->set_axis_param(idx + 5, "TABLE", 1);

		// 形态位
		ZController->set_axis_param(idx + 1, "TABLE", -1);

		// 终点
		idxList = std::vector<int>(12, idx + 22 + 12 + 12);
		for (size_t i = 0; i < idxList.size(); ++i) {
			idxList[i] += i;
		}
		ZController->set_axis_param(idxList, "TABLE", end);
		ZController->set_axis_param(idx + 22 + 36 + 2, "TABLE", 0);

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				ZController->set_axis_param(idx + 280 + i, "TABLE", 1);
		}

		return 0;
	}

	int FSAIRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {

		return 0;

	}
	int FSAIRobot::moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		int idx = get_point_idx_base();
		std::vector<int> idxList;
		std::vector<float> value;
		// 运动类型
		ZController->set_axis_param(idx + 5, "TABLE", 0);

		// 形态位
		ZController->set_axis_param(idx + 1, "TABLE", -1);

		// 终点
		idxList = std::vector<int>(12, idx + 22 + 12 + 12);
		for (size_t i = 0; i < idxList.size(); ++i) {
			idxList[i] += i;
		}
		ZController->set_axis_param(idxList, "TABLE", end);
		ZController->set_axis_param(idx + 22 + 36 + 2, "TABLE", 1);

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				ZController->set_axis_param(idx + 280 + i, "TABLE", 1);
		}

		return 0;
	}
	
	int FSAIRobot::moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		return 0;
	}
	int FSAIRobot::moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {

		int idx = get_point_idx_base();
		std::vector<int> idxList;
		std::vector<float> value;
		// 运动类型
		ZController->set_axis_param(idx + 5, "TABLE", 2);

		// 形态位
		ZController->set_axis_param(idx + 1, "TABLE", -1);

		// 中间点
		idxList = std::vector<int>(12, idx + 22 + 12);
		for (size_t i = 0; i < idxList.size(); ++i) {
			idxList[i] += i;
		}
		ZController->set_axis_param(idxList, "TABLE", mid);
		ZController->set_axis_param(idx + 22 + 36 + 1, "TABLE", 1);

		// 终点
		idxList = std::vector<int>(12, idx + 22 + 12 + 12);
		for (size_t i = 0; i < idxList.size(); ++i) {
			idxList[i] += i;
		}
		ZController->set_axis_param(idxList, "TABLE", end);
		ZController->set_axis_param(idx + 22 + 36 + 2, "TABLE", 1);

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				ZController->set_axis_param(idx + 280 + i, "TABLE", 1);
		}

		return 0;
	}

	/* *************************** 运动设置 *************************** */
	int FSAIRobot::send_line_num(int axis, const SingleTrajectory &curTraj) {

		int ret = 0;
		int stateIdxBase = get_point_idx_base();
		ret = ZController->set_axis_param(stateIdxBase, "TABLE", curTraj.lineNum);

		//int tableIdx = get_state_idx_base() + 3;
		//ret = ZController->set_axis_param(tableIdx, "TABLE", curTraj.lineNum, axis);

		LOG4CPLUS_INFO(RobotLog::getLogger(),
			"R" << aliasId << " send point with line num: " << cmdNum
		);

		return ret;

	}

	int FSAIRobot::set_previous_trajectory(const SingleTrajectory& preTraj) {
		auto pnt = preTraj.mainPoint;

		int idx = 160000;
		std::vector<int> idxList;

		// 起点
		idxList = std::vector<int>(12, idx + 22);
		for (size_t i = 0; i < idxList.size(); ++i) {
			idxList[i] += i;
		}
		ZController->set_axis_param(idxList, "TABLE", pnt);

		// 起点为基坐标系运动
		if (preTraj.isBaseMotion()) {
			ZController->set_axis_param(idx + 64, "TABLE", 1);
		}
		else {
			// 轴屏蔽
			std::vector<int> mask(9, 1);
			auto trajAxisMask = preTraj.get_axisMask();
			for (size_t i = 0; i < mask.size(); ++i) {
				// 机器人指定的屏蔽
				if (axisMask.count(i) > 0)
					mask[i] = -1;
				// 轨迹指定的屏蔽
				for (size_t j = 0; j < trajAxisMask.size(); ++j) {
					if (i == trajAxisMask[j])
						mask[i] = -1;
				}
			}
			for (size_t i = 0; i < mask.size(); ++i) {
				if (mask[i] < 0)
					ZController->set_axis_param(idx + 268 + i, "TABLE", 1);
			}
		}


		// 起点类型
		if (preTraj.isJoint()) {
			ZController->set_axis_param(idx + 22 + 36 + 0, "TABLE", 0);
		}
		else {
			ZController->set_axis_param(idx + 22 + 36 + 0, "TABLE", 1);
		}

		return 0;
	}


	/* *************************** 连续运动 *************************** */
	int FSAIRobot::execute_single_joint() {
		int ret = 0;
		// 前一条轨迹
		auto preTraj = trajectory.get_preTraj();
		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();

		std::vector<int> axis = get_execute_axis();

		// 运动类型检查

		// 获取节点目标位置
		auto pnt = curTraj.mainPoint;

		// 轴屏蔽
		std::vector<int> mask(9, 1);
		auto trajAxisMask = curTraj.get_axisMask();
		for (size_t i = 0; i < mask.size(); ++i) {
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
				maskF.push_back(static_cast<float>(i));
		}

		// 设置速度
		ZController->set_axis_param(160000 + 6, "TABLE", curTraj.get_speed());
		// 加速度
		ZController->set_axis_param(160000 + 8, "TABLE", 100);
		// 设置平滑度
		ZController->set_axis_param(160000 + 4, "TABLE", curTraj.get_smooth());

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveJABS: " << vector_to_string(pnt));
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
			<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : ""));

		// 开始记录位置
		save_task_status(true, 1);

		// 设置上条轨迹类型
		auto beg = preTraj.get_mainPoint();
		set_previous_trajectory(preTraj);
		// 更新轨迹编号
		trajectory.trajList.front().lineNum = ++cmdNum;
		// 下发轨迹序号
		send_line_num(axis[0], trajectory.get_curTraj());

		// 下发轨迹
		ret = moveJABS(axis, beg, pnt, mask);
		if (ret != 0)
			return ret;

		// 停止记录位置
		save_task_status(false, 1);

		// 轨迹出栈
		if (ret == 0) {
			//trajectory.set_current_line_num(cmdNum);
			trajectory.next();
		}
		else {
			return -1;
		}

		return 0;
	}

	int FSAIRobot::execute_single_cartesian() {

		int stateIdxBase = get_state_idx_base();
		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();
		auto preTraj = trajectory.get_preTraj();

		int ret = 0;
		std::vector<int> axis = get_execute_axis();

		// 获取节点目标位置
		auto curPoint = curTraj.mainPoint;
		auto prePoint = preTraj.mainPoint;
		auto midPoint = curTraj.auxPoint;

		// 轴屏蔽
		std::vector<int> mask(9, 1);
		if (!curTraj.isBaseMotion()) {
			auto trajAxisMask = curTraj.get_axisMask();
			for (size_t i = 0; i < mask.size(); ++i) {
				// 机器人指定的屏蔽
				if (axisMask.count(i) > 0)
					mask[i] = -1;
				// 轨迹指定的屏蔽
				for (size_t j = 0; j < trajAxisMask.size(); ++j) {
					if (i == trajAxisMask[j])
						mask[i] = -1;
				}
			}
		}
		std::vector<float> maskF;
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				maskF.push_back(static_cast<float>(i));
		}

		if (curTraj.isArc()) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveCABS: " << vector_to_string(curPoint)
				<<";\nmid: " << vector_to_string(midPoint));
		}
		else {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveLABS: " << vector_to_string(curPoint));
		}
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
			<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : "")     // 轴掩码
			<< ". traj dist: " << trajectory.get_dist()
		);

		// 轨迹点维度与驱动轴维度的较小值
		size_t num = (std::min)(curPoint.size(), axis.size());

		// 修改摆焊参数
		update_swing_config();
		// 修改焊接参数
		update_welder_config();
		// 修改跟踪参数
		update_track_config();

		// 设置速度
		ZController->set_axis_param(160000 + 6, "TABLE", curTraj.get_speed());
		ZController->set_axis_param(160000 + 7, "TABLE", 200);
		// 加速度
		ZController->set_axis_param(160000 + 8, "TABLE", 500);
		ZController->set_axis_param(160000 + 9, "TABLE", 500);
		// 设置平滑度
		ZController->set_axis_param(160000 + 4, "TABLE", curTraj.get_smooth());

		// 开始记录位置
		save_task_status(true, 1);

		// 设置上条轨迹类型
		set_previous_trajectory(preTraj);
		// 更新轨迹编号
		trajectory.trajList.front().lineNum = ++cmdNum;
		// 下发轨迹序号
		send_line_num(axis[0], trajectory.get_curTraj());

		// 本体坐标系运动
		if (curTraj.isBaseMotion()) {
			ZController->set_axis_param(160000 + 65, "TABLE", 1);
			ZController->set_axis_param(160000 + 66, "TABLE", 1);
		}

		// 下发轨迹
		if (curTraj.isArc()) {
			moveCABS(axis, prePoint, midPoint, curPoint, 0, mask);
		}
		else if (curTraj.isLine()) {
			moveLABS(axis, prePoint, curPoint, mask);
		}

		// 停止记录位置
		save_task_status(false, 1);

		// 下发异常
		if (ret == 0) {
			trajectory.next();
		}
		else {
			return -1;
		}

		return ret;
	}

	int FSAIRobot::set_ready_for_consistent_traj(int& state) {
		if (trajectory.trajectory_loaded())
			return 0;

		// 开始执行新轨迹
		if (trajectory.get_preTraj().trajType == TrajType::None) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " new traj begin point: " << vector_to_string(robotStatus.jPos));
			SingleTrajectory preTraj;
			preTraj.mainPoint = robotStatus.jPos;
			preTraj.trajType = TrajType::Joint;
			trajectory.set_preTraj(preTraj);
		}

		return 0;
	}

	int FSAIRobot::consistent_traj_ready(int& state) {

		int ret = 0;
		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();
		// 获取上一条轨迹
		auto preTraj = trajectory.get_preTraj();

		// 第一条轨迹未就绪
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

	int FSAIRobot::remain_buffer_free() {
		int idx = get_cmd_idx_base() + 29990;
		std::vector<float> value;

		ZController->get_register(idx, 2, value, 0);
		//ZController->get_axis_param(idx, "TABLE", value);

		return (std::fabs(value[0]) + std::fabs(value[1]) < 1e-2) ? 1 : 0;
	}

	int FSAIRobot::separate_trajectory() {
		return 0;
	}

	/* *************************** 固定运动 *************************** */
	int FSAIRobot::save_task_status(bool enable, int inBuffer) {
		int stateIdxBase = get_state_idx_base();
		int ret = 0;
		//std::vector<int> axis = get_execute_axis();
		//ret = ZController->set_axis_param(stateIdxBase + 100, "TABLE", enable, axis[0]);

		if (inBuffer < 0) {
			ret = ZController->set_axis_param(stateIdxBase + 100, "TABLE", enable);
		}
		else {
			if (enable) {
				begRegister.add_buffer(stateIdxBase + 100, 1);
			}
			else {
				endRegister.add_buffer(stateIdxBase + 100, 0);
			}
		}

		return ret;
	}


	int FSAIRobot::write_buffer_register(int flag) {

		int idx = get_point_idx_base();
		if (flag == 0) {
			while (!begRegister.empty()) {
				std::pair<int, float> pair;
				begRegister.pop(pair);

				ZController->set_axis_param(idx + 67 + begRegister.get_num() - 1, "TABLE", pair.first);
				ZController->set_axis_param(idx + 167 + begRegister.get_num() - 1, "TABLE", pair.second);
			}
		}
		else {
			while (!endRegister.empty()) {
				std::pair<int, float> pair;
				endRegister.pop(pair);

				ZController->set_axis_param(idx + 127 + endRegister.get_num() - 1, "TABLE", pair.first);
				ZController->set_axis_param(idx + 227 + endRegister.get_num() - 1, "TABLE", pair.second);
			}
		}

		return 0;
	}

	/* *************************** 自定义功能 *************************** */
	int FSAIRobot::read_saved_status(RobotStatus& status) {

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
		//auto rotMat = robotConfig.get_slave_calibratino_mat();
		//auto euler = std::vector<float>(status.cPosRaw.begin() + 3, status.cPosRaw.begin() + 6);
		//Eigen::Matrix3f curMat = Eigen::AngleAxisf(euler[2] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
		//	Eigen::AngleAxisf(euler[1] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
		//	Eigen::AngleAxisf(euler[0] * DT_PI / 180, Eigen::Vector3f::UnitX()).matrix();
		//auto afterEuler = (rotMat * curMat).eulerAngles(2, 1, 0);
		//for (size_t i = 0; i < 3; ++i) {
		//	status.cPos[3 + i] = afterEuler[2 - i] * 180 / DT_PI;
		//}

		return 0;
	}


	int FSAIRobot::task_pause() {

		int stateIdxBase = get_state_idx_base();
		//ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 2 });
		ZController->set_axis_param({ stateIdxBase + 61 }, "TABLE", { 1 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task pause.");
		return 0;

	}


	int FSAIRobot::task_resume() {

		// 不在自动模式
		if (robotStatus.autoMode <= 0)
			return 1;

		int stateIdxBase = get_state_idx_base();
		//ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 1 });
		ZController->set_axis_param({ stateIdxBase + 62 }, "TABLE", { 1 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task resume.");
		return 0;

	}


	int FSAIRobot::task_stop() {

		int stateIdxBase = get_state_idx_base();

		// 轨迹清空
		trajectory.clear();
		// 已下发轨迹清空
		trajHistory.clear();

		//// 停止记录位置
		//save_task_status(false, -1);

		//// 清除上位机异常码
		//reset_upperStatus(-1);

		//int stateIdxBase = get_cmd_idx_base();
		//// 暂停
		//ZController->set_axis_param({ stateIdxBase + 23997 }, "TABLE", { 1 });
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// 轨迹序号复位
		reset_line_num();

		ZController->set_axis_param({ stateIdxBase + 63 }, "TABLE", { 1 });

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " task stop.");

		return 0;

	}


	int FSAIRobot::emergency_stop() {

		int stateIdxBase = get_state_idx_base();
		// 轨迹清空
		trajectory.clear();
		// 已下发轨迹清空
		trajHistory.clear();
		//ZController->set_axis_param({ stateIdxBase + 52 }, "TABLE", { 3 });
		ZController->set_axis_param({ stateIdxBase + 60 }, "TABLE", { 1 });

		// 上位机下发停止
		set_upperStatus(0x08);

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " emergency stop.");
		return 0;

	}


	int FSAIRobot::device_operation() {
		return 0;
	}


	int FSAIRobot::execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag) {

		int stateIdxBase = get_state_idx_base();
		std::vector<int> axis = get_execute_axis();

		int delayNum = 0;
		for (const auto& action : actionList) {
			auto type = action.first;
			auto param = action.second;

			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Move_Action " << type << (param.size() > 0 ? ": " : "") << vector_to_string(param));

			// 仅延时
			if (type == 1 && param[0] < 0) {
				ZController->set_axis_param(flag == 0 ? (get_point_idx_base() + 21) : (get_point_idx_base() + 20), "TABLE", param[2]);
				delayNum++;
				continue;
			}
			// 寻位动作，行号不返回
			if (type == 5) {
				int idx = get_point_idx_base();
				ZController->set_axis_param(idx + 267, "TABLE", 1);
				delayNum++;
			}

			// 完成标志位复位
			if (flag == 0)
				begRegister.add_buffer(stateIdxBase + 350, 0);
			else
				endRegister.add_buffer(stateIdxBase + 350, 0);


			// 下发运动参数
			if (param.size() > 0) {
				std::vector<int> idx(param.size(), stateIdxBase + 301);
				for (size_t i = 0; i < param.size(); ++i) {
					idx[i] += i;
				}

				//ZController->set_axis_param(idx, "TABLE", param, axis[0]);
				if (flag == 0)
					begRegister.add_buffer(idx, param);
				else
					endRegister.add_buffer(idx, param);
			}

			// 下发运动
			if (flag == 0)
				begRegister.add_buffer(stateIdxBase + 300, type);
			else
				endRegister.add_buffer(stateIdxBase + 300, type);

			// 等待运动结束
			//ZController->move_wait(axis[0], "TABLE", stateIdxBase + 350, 0, 1);

		}

		// 等待运动结束
		if (actionList.size() > delayNum) {
			ZController->set_axis_param(flag == 0 ? get_point_idx_base() + 19 : get_point_idx_base() + 18, "TABLE", 1);
		}

		return 0;
	}


	int FSAIRobot::process_after_send_traj() {

		int idx = get_point_idx_base();

		// 运动前缓冲
		write_buffer_register(0);
		begRegister.clear();
		// 运动后缓冲
		write_buffer_register(1);
		endRegister.clear();

		// 点位插入完成
		ZController->set_axis_param(idx - 10, "TABLE", 1);

		return 0;
	}


	int FSAIRobot::update_swing_config() {

		auto curTraj = trajectory.get_curTraj();
		Weave waveCfg = deserialize_Weave(curTraj.get_appendix());

		std::vector<int> idx(8, 160000 + 10);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}
		std::vector<float> value(idx.size(), 0.0);

		value[0] = waveCfg.Id;
		value[1] = (waveCfg.Id == 0 || (waveCfg.Dwell_left + waveCfg.Dwell_right) < 1e-2) ?  0 : waveCfg.Dwell_type;
		value[2] = waveCfg.LeftWidth;
		value[3] = waveCfg.RightWidth;
		value[5] = waveCfg.Freq;
		value[6] = waveCfg.Dwell_left;
		value[7] = waveCfg.Dwell_right;

		ZController->set_axis_param(idx, "TABLE", value);

		if (waveCfg.Id > 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Update swing config: " <<
				vector_to_string(serialize_Weave(waveCfg).second, 2)
			);
		}

		return 0;
	}


	int FSAIRobot::update_track_config() {

		auto curTraj = trajectory.get_curTraj();
		Track trackCfg = deserialize_Track(curTraj.get_appendix());

		int stateIdxBase = get_state_idx_base();
		std::vector<int> axis = get_execute_axis();
		std::vector<float> config = serialize_Track(trackCfg).second;
		size_t configTableStart = stateIdxBase + 180;
		int ret = 0;

		// 跟踪未使能
		if (trackCfg.Id > 0) {
			// 电弧跟踪标志位，区分电弧跟踪和线激光跟踪
			//ZController->set_axis_param(stateIdxBase + 150, "TABLE", trackCfg.Id, axis[0]);
			begRegister.add_buffer(stateIdxBase + 150, trackCfg.Id);
			// 跟踪开启和关闭
			begRegister.add_buffer(get_cmd_idx_base() + 24100, 1);
			endRegister.add_buffer(get_cmd_idx_base() + 24101, 1);
			

			// 下发跟踪参数
			for (size_t i = 0; i < config.size(); ++i) {
				//ZController->set_axis_param(configTableStart + i, "TABLE", config[i], axis[0]);
				begRegister.add_buffer(configTableStart + i, config[i]);
			}

			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId <<
				" Update track config: " << vector_to_string(serialize_Track(trackCfg).second, 2)
			);
		}
		else {
			//ZController->set_axis_param(stateIdxBase + 187, "TABLE", trackCfg.Id, axis[0]);
			begRegister.add_buffer(stateIdxBase + 187, trackCfg.Id);
		}

		return 0;
	}


	int FSAIRobot::update_welder_config() {

		auto curTraj = trajectory.get_curTraj();
		Arc_WeldingParaItem weldCfg = deserialize_Arc_WeldingParaItem(curTraj.get_appendix());

		// 不起弧，无需修改焊接参数
		if (weldCfg.Id <= 0)
			return 1;

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
		//ZController->set_axis_param(tableList, "TABLE", data, get_execute_axis()[0]);
		begRegister.add_buffer(tableList, data);
		// 变工艺使能
		//ZController->set_axis_param(stateBase + 170, "TABLE", 1, get_execute_axis()[0]);
		begRegister.add_buffer(stateBase + 170, 1);

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId <<
			" Update welder config: " << vector_to_string(serialize_Arc_WeldingParaItem(weldCfg).second, 2)
		);

		return 0;
	}


} // namespace ZMotionRobot
