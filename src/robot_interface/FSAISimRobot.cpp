
#include "robot_interface/FSAISimRobot.h"

#include "fsai_simulator/CMotion.h"
#include "RobotLogger.h"
#include <iostream>

namespace FSAIRobotInterface {
	// 仿真器数据
	static CMotion m_Motion;
	static FSData m_dBase[100];
	static double m_simTotalTime = 0.0;

	//! 获取下发指令轴号，主要用于确定运动主轴
	std::vector<int> FSAISimRobot::get_execute_axis() {
		return {0, 1, 2, 3, 4, 5, 6, 7};
	}

	// 机器人状态
	int FSAISimRobot::update_rt_robot_status() {
		RobotStatus tmp;

		// 关节位置
		tmp.jPos = std::vector<float>(simStatus.jPos.size(), 0.0);
		for (int i = 0; i < 9; ++i) {
			tmp.jPos[i] = simStatus.jPos[i];
		}
		// 手自动模式
		tmp.autoMode = static_cast<int>(simStatus.autoMode);

		// 空间点位
		tmp.cPos = tmp.jPos;
		AKDLL_JPosToCPos(simStatus.jPos.data(), CP_ToolToB, simStatus.cPos.data());
		for (int i = 0; i < 6; ++i) {
			tmp.cPos[i] = simStatus.cPos[i];
		}

		// 机器人状态
		tmp.lowerStatus = EDLL_GetAutoStatus();

		// 加锁
		std::lock_guard<std::mutex> lock(mtxMotion);
		robotStatus = tmp;

		return 0;
	}
    
	int FSAISimRobot::get_all_robot_status(RobotStatus& status) {
		{
			std::lock_guard<std::mutex> lock(mtxMotion);
			status = robotStatus;
		}

		// 仿真时间
		status.curInterpTime = m_simTotalTime;

		return 0;
	}

	int FSAISimRobot::get_register_config(RobotConfig& config) {
		config = robotConfig;
		return 0;
	}

	int FSAISimRobot::read_register_config() {
		int ret = 0;

		// 连杆方向
		robotConfig.linkLength = { 430, 163.598, 821.777, 210.852, 1029.198, 115 };
		// 附加轴标定
		robotConfig.auxCalbration = { -90,0,0, 0,90,0, 0,0,0 };
		// 工具坐标系
		robotConfig.tcpPose = { 88.677, -2.591, 728.591, 0, 0.628, 0 };
		// 正负限位
		robotConfig.jointSupremum = { 165, 150, 170, 180, 165, 360, 200000000, 200000000, 200000000 };
		robotConfig.jointInfimum = { -165, -90, -90, -180, -135, -360, -200000000, -200000000, -200000000 };
		// 关节最大速度
		robotConfig.maxJointSpeedAuto = { 80,80,80,120,100,200,600,800,1000 };

		return ret;
	}

	int FSAISimRobot::write_register_config(const RobotConfig& config) {
		int ret = 0;

		// 拷贝到当前程序
		robotConfig = config;

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " write register config.");

		return ret;
	}

	int FSAISimRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		int num = 0;

		// 运动类型
		m_dBase[num].m_MoveType = Joint;

		// 形态位
		m_dBase[num].m_Estate.J1Front = -1; m_dBase[num].m_Estate.J3UP = m_dBase[num].m_Estate.J5UP = 0;

		// 终点
		for (int i = 0; i < 12; ++i) {
			m_dBase[num].m_PosData.EndPos.dPos[i] = i < end.size() ? end[i] : 0.0;
		}
		m_dBase[num].m_PosData.EndPos.dPointType = CP_Joint;

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				m_dBase[num].BlockAxis[12 + i] = 1;
		}

		return 0;
	}

	int FSAISimRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		int num = 0;
		// 运动类型
		m_dBase[num].m_MoveType = Line;

		// 形态位
		m_dBase[num].m_Estate.J1Front = -1; m_dBase[num].m_Estate.J3UP = m_dBase[num].m_Estate.J5UP = 0;

		// 终点
		for (int i = 0; i < 12; ++i) {
			m_dBase[num].m_PosData.EndPos.dPos[i] = i < end.size() ? end[i] : 0.0;
		}
		m_dBase[num].m_PosData.EndPos.dPointType = CP_Space;

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				m_dBase[num].BlockAxis[12 + i] = 1;
		}
		return 0;
	}

	int FSAISimRobot::moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		int num = 0;
		// 运动类型
		m_dBase[num].m_MoveType = Circle;

		// 形态位
		m_dBase[num].m_Estate.J1Front = -1; m_dBase[num].m_Estate.J3UP = m_dBase[num].m_Estate.J5UP = 0;

		// 中间点
		for (int i = 0; i < 12; ++i) {
			m_dBase[num].m_PosData.MidPos.dPos[i] = i < mid.size() ? mid[i] : 0.0;
		}
		m_dBase[num].m_PosData.MidPos.dPointType = CP_Space;

		// 终点
		for (int i = 0; i < 12; ++i) {
			m_dBase[num].m_PosData.EndPos.dPos[i] = i < end.size() ? end[i] : 0.0;
		}
		m_dBase[num].m_PosData.EndPos.dPointType = CP_Space;

		// 终点轴号屏蔽
		for (size_t i = 0; i < mask.size(); ++i) {
			if (mask[i] < 0)
				m_dBase[num].BlockAxis[12 + i] = 1;
		}

		return 0;
	}

	int FSAISimRobot::move_compensate(const std::vector<float>& det) {
		return 0;
	}

	/**
	* @brief  设置手动速度比率
	* @param  ratio    速度比率(0-100)
	* @return 设置状态: 0 - 设置成功; 1 - 未处于手动模式
	*/
	int FSAISimRobot::set_manual_speed(float ratio) {
		return 0;
	}

	// 自动任务
	int FSAISimRobot::update_swing_config() {
		int num = 0;
		auto curTraj = trajectory.get_curTraj();
		Weave waveCfg = deserialize_Weave(curTraj.get_appendix());

		// 基础摆焊参数
		m_dBase[num].m_WavePara.WaveFlage = waveCfg.Id;
		m_dBase[num].m_WavePara.WaveDiscontinue = (waveCfg.Id == 0 || (waveCfg.Dwell_left + waveCfg.Dwell_right) < 1e-2) ? 0 : waveCfg.Dwell_type;
		m_dBase[num].m_WavePara.Amplitude_UP = waveCfg.LeftWidth;
		m_dBase[num].m_WavePara.Amplitude_DOWN = waveCfg.RightWidth;
		m_dBase[num].m_WavePara.frequency = waveCfg.Freq;
		m_dBase[num].m_WavePara.TimeStop[0] = waveCfg.Dwell_left;
		m_dBase[num].m_WavePara.TimeStop[1] = waveCfg.Dwell_right;

		// 补充摆焊参数
		// 正弦摆
		if (waveCfg.Shape == 0) {
			m_dBase[num].m_WavePara.WaveType = 0;
		}
		// 三角摆
		else if (waveCfg.Shape == 1) {
			m_dBase[num].m_WavePara.WaveType = 4;
		}
		// L 摆
		else if (waveCfg.Shape == 2) {
			m_dBase[num].m_WavePara.WaveType = 3;
		}
		// 钟摆
		else if (waveCfg.Shape == 3) {
			m_dBase[num].m_WavePara.WaveType = 2;
		}
		// 斜正弦
		else if (waveCfg.Shape == 4) {
			m_dBase[num].m_WavePara.WaveType = 1;
		}
		m_dBase[num].m_WavePara.TimeStop[2] = waveCfg.Dwell_center;
		m_dBase[num].m_WavePara.LRSwingTheta[0] = waveCfg.Angle_Ltype_top;
		m_dBase[num].m_WavePara.LRSwingTheta[1] = waveCfg.Angle_Ltype_btm;
		m_dBase[num].m_WavePara.DirectionTheta = waveCfg.AzimuthAngle;
		m_dBase[num].m_WavePara.ElevationTheta = waveCfg.Angle_lead;
		//m_dBase[num].m_WavePara.WaveTheta = waveCfg.Angle_lead;

		if (waveCfg.Id > 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Update swing config: " <<
				vector_to_string(serialize_Weave(waveCfg).second, 2)
			);
		}
		return 0;
	}
	int FSAISimRobot::update_track_config() {
		return 0;
	}
	int FSAISimRobot::update_welder_config() {
		return 0;
	}
	int FSAISimRobot::get_remain_buffer() {
		return 0;
	}

	int FSAISimRobot::push_new_trajectory(DiscreteTrajectory trajList) {
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
		for (auto& traj : trajList.trajList) {
			// 1. 欧拉角修正
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

	int FSAISimRobot::execute_single_joint() {
		int Buf = 0, num = Buf;

		int ret = 0;
		// 前一条轨迹
		auto preTraj = trajectory.get_preTraj();
		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();

		std::vector<int> axis = get_execute_axis();

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
		m_dBase[num].Vel[0] = curTraj.get_speed();
		// 加速度
		m_dBase[num].Acc[0] = 100;
		// 设置平滑度
		m_dBase[num].dRoundRadius = curTraj.get_smooth();

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveJABS: " << vector_to_string(pnt));
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
			<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth()
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : "")
			<< ", notifyEnable: " << curTraj.notifyEnable);

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
			trajectory.next();
		}
		else {
			return -1;
		}

		//基于序号将dBase[Buf]赋值到Table
		m_Motion.MBaseToTData(m_dBase, Buf, m_Motion.m_TableData);
		return 0;
	}
	
	int FSAISimRobot::execute_single_cartesian() {
		int ret = 0, num = 0, Buf = 0;
		std::vector<int> axis = get_execute_axis();

		// 获取当前轨迹
		auto curTraj = trajectory.get_curTraj();
		auto preTraj = trajectory.get_preTraj();
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
				<< ";\nmid: " << vector_to_string(midPoint));
		}
		else {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " MoveLABS: " << vector_to_string(curPoint));
		}
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId
			<< " Trajectory config: " << curTraj.get_speed() << ", " << curTraj.get_smooth() << ", " << curTraj.taskId
			<< (maskF.size() > 0 ? (". Axis mask: " + vector_to_string(maskF)) : "")     // 轴掩码
			<< ". traj dist: " << trajectory.get_dist() << ", notifyEnable: " << curTraj.notifyEnable << ", rewriteId: " << curTraj.rewriteId
		);

		//// 修改摆焊参数
		//update_swing_config();
		//// 修改焊接参数
		//update_welder_config();
		//// 修改跟踪参数
		//update_track_config();

		// 设置速度
		m_dBase[num].Vel[0] = curTraj.get_speed();
		m_dBase[num].Vel[1] = 200;
		// 加速度
		m_dBase[num].Acc[0] = 500;
		m_dBase[num].Acc[1] = 500;
		// 设置平滑度
		m_dBase[num].dRoundRadius = curTraj.get_smooth();

		// 开始记录位置
		save_task_status(true, 1);

		// 设置上条轨迹类型
		set_previous_trajectory(preTraj);
		// 更新轨迹编号
		trajectory.trajList.front().lineNum = ++cmdNum;
		// 下发轨迹序号
		send_line_num(0, trajectory.get_curTraj());

		// 本体坐标系运动
		if (curTraj.isBaseMotion()) {
			m_dBase[num].m_PosData.MidPos.PointOnBaseFlage = true;
			m_dBase[num].m_PosData.EndPos.PointOnBaseFlage = true;
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

		//基于序号将dBase[Buf]赋值到Table
		m_Motion.MBaseToTData(m_dBase, Buf, m_Motion.m_TableData);
		return ret;
	}

	// 设置运动行号
	int FSAISimRobot::send_line_num(int axis, const SingleTrajectory &curTraj) {
		int ret = 0, num = 0;

		m_dBase[num].LineNum = curTraj.lineNum;
		LOG4CPLUS_INFO(RobotLog::getLogger(),
			"R" << aliasId << " send point with line num: " << cmdNum
		);

		return ret;
	}

	// 剩余缓冲检测
	int FSAISimRobot::remain_buffer_free() {
		return (m_Motion.m_PosToTableFlage);
	}

	// 一致性轨迹预处理，可以连续下发的轨迹
	int FSAISimRobot::set_ready_for_consistent_traj(int& state) {
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

	// 一致性轨迹就绪
	int FSAISimRobot::consistent_traj_ready(int& state) {

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

	int FSAISimRobot::separate_trajectory() {
		return 0;
	}

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief  读取保存点位
	*/
	int FSAISimRobot::read_saved_status(RobotStatus& status) {
		return 0;
	}

	int FSAISimRobot::get_local_world_dpos(std::vector<float>& dpos) {
		return 0;
	}
	int FSAISimRobot::cpos_base_to_world(std::vector<float>& cPos) {
		return 0;
	}

	int FSAISimRobot::switch_auto(bool enableAuto) {

		if (enableAuto) {
			m_Motion.MotionStateAuto();
			simStatus.autoMode = true;
		}
		else {
			m_Motion.MotionStateManual();
			simStatus.autoMode = false;
		}

		return 0;
	}
	int FSAISimRobot::switch_enable(bool enable) {
		return 0;
	}

	int FSAISimRobot::reset_line_num() {
		return 0;
	}
	int FSAISimRobot::set_jog_type(int type) {
		if (type == 0)
			m_Motion.JogeModeJoint();
		else if (type == 1)
			m_Motion.JogeModeSpace();
		return 0;
	}
	int FSAISimRobot::jog_moving(int type, int idx, int dir, int move) {
		static int jogDir = 0;
		set_jog_type(type);

		if (dir > 0) {
			m_Motion.JopePositive(idx);
			jogDir = 1;
		}
		else if (dir < 0) {
			m_Motion.JopeNegative(idx);
			jogDir = -1;
		}
		else {
			if (jogDir > 0) {
				m_Motion.JopePositive(idx);
			}
			else if (jogDir < 0) {
				m_Motion.JopeNegative(idx);
			}
			jogDir = 0;
		}

		return 0;
	}
	int FSAISimRobot::save_task_status(bool enable, int inBuffer) {
		return 0;
	}

	int FSAISimRobot::task_pause() {
		return 0;
	}
	int FSAISimRobot::task_resume() {
		return 0;
	}
	int FSAISimRobot::task_stop() {
		return 0;
	}
	int FSAISimRobot::emergency_stop() {
		return 0;
	}

	// 设备操作
	int FSAISimRobot::device_operation() {
		return 0;
	}

	int FSAISimRobot::execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag) {
		return 0;
	}
	int FSAISimRobot::process_after_send_traj() {
		//// 运动前缓冲
		//write_buffer_register(0);
		//begRegister.clear();
		//// 运动后缓冲
		//write_buffer_register(1);
		//endRegister.clear();

		// 点位插入完成
		//ZController->set_axis_param(idx - 10, "TABLE", 1);

		// 等待点位插入
		m_Motion.m_PosToTableFlage = false;

		return 0;
	}

	/* *************************** 获取运动轴号 *************************** */
	// 设置上条轨迹类型
	int FSAISimRobot::set_previous_trajectory(const SingleTrajectory& preTraj) {
		auto pnt = preTraj.mainPoint;
		int num = 0;

		// 起点
		for (int i = 0; i < 12; ++i) {
			m_dBase[num].m_PosData.StartPos.dPos[i] = i < pnt.size() ? pnt[i] : 0.0;
		}

		// 起点为基坐标系运动
		if (preTraj.isBaseMotion()) {
			m_dBase[num].m_PosData.StartPos.PointOnBaseFlage = true;
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
					m_dBase[num].BlockAxis[i] = 1;
			}
		}


		// 起点类型
		if (preTraj.isJoint()) {
			m_dBase[num].m_PosData.StartPos.dPointType = CP_Joint;
		}
		else {
			m_dBase[num].m_PosData.StartPos.dPointType = CP_Space;
		}

		return 0;
	}

	/* *************************** 初始化 *************************** */
	FSAISimRobot::FSAISimRobot() {
		switch_robot_mode(0);

        simThreadDone.store(true); 
        // 开启插补线程，执行点位插补
        std::thread sim_thread(&FSAISimRobot::sim_thread, this);
        sim_thread.detach();
		// 开启插点线程，将 m_Motion 点位存入插补器
		std::thread myThread2(&CMotion::InsertPos, ref(m_Motion));
		myThread2.detach();	

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		// 开始信号
		m_Motion.StartMove();
	}
	FSAISimRobot::~FSAISimRobot() {
	}

	// 写入缓冲寄存器
	int FSAISimRobot::write_buffer_register(int flag) {
		return 0;
	}

    void FSAISimRobot::sim_thread() {
        simThreadDone.store(false);
        // 获取当前时间戳
        auto start = std::chrono::steady_clock::now();
        // 下次唤醒时间
        auto wakeUpTime = start;
        // 线程周期(ms)
        long long duration = static_cast<int>(simStatus.m_timecycle * 1000);

        while (!simThreadDone) {
            // 设置下次唤醒时间
            wakeUpTime += std::chrono::milliseconds(duration);
            auto now = std::chrono::steady_clock::now();
			
			double CheckIOflage = 0;
			double TableID[50] = { 0 }, TableValue[50] = { 0 };
			std::vector<double> JPos(12, 0);
			// 插补
			m_Motion.UpdateJPos(JPos.data(), CheckIOflage, TableID, TableValue);
			simStatus.jPos = JPos;
			// 更新插补时间
			m_simTotalTime = EDLL_GetTimeAll();

            // 周期时间耗尽
            if (now > wakeUpTime) {
                auto detTime = now - wakeUpTime;
                while (now > wakeUpTime)
                    wakeUpTime += std::chrono::milliseconds(duration);
            }
            // 休眠
            else {
                std::this_thread::sleep_until(wakeUpTime);
            }
        }
    }

	int FSAISimRobot::switch_robot_mode(int type) {
		bool onlyCalcTime = type;

		RobotConfig config;
		read_register_config();
		get_register_config(config);

		// 插补周期8ms
		simStatus.m_timecycle = 8e-3;
		// DH参数
		double m_dDHPara[6][4] = { 0 };
		m_dDHPara[0][0] = config.linkLength[1];		m_dDHPara[0][1] = config.linkLength[0];			m_dDHPara[0][2] = -90;
		m_dDHPara[1][0] = config.linkLength[2];		m_dDHPara[1][1] = 0;			                m_dDHPara[1][2] = 0;
		m_dDHPara[2][0] = -config.linkLength[3];	m_dDHPara[2][1] = 0;			                m_dDHPara[2][2] = 90;
		m_dDHPara[3][0] = 0;			            m_dDHPara[3][1] = config.linkLength[4];		    m_dDHPara[3][2] = -90;
		m_dDHPara[4][0] = 0;			            m_dDHPara[4][1] = 0;			                m_dDHPara[4][2] = 90;
		m_dDHPara[5][0] = 0;			            m_dDHPara[5][1] = config.linkLength[5];			m_dDHPara[5][2] = 0;
		// 附加轴标定坐标系
		double DGAxisCPos[3][6] = { {0,0,0,config.auxCalbration[2],config.auxCalbration[1],config.auxCalbration[0]},
		                            {0,0,0,config.auxCalbration[5],config.auxCalbration[4],config.auxCalbration[3]},
		                            {0,0,0,config.auxCalbration[8],config.auxCalbration[7],config.auxCalbration[6]} };
		// 工具坐标系
		double ToolCPos[20][6] = { 0 };
		ToolCPos[0][0] = config.tcpPose[0];	ToolCPos[0][1] = config.tcpPose[1];	          ToolCPos[0][2] = config.tcpPose[2];
		ToolCPos[0][3] = config.tcpPose[5]; ToolCPos[0][4] = config.tcpPose[4]*180/DT_PI; ToolCPos[0][5] = config.tcpPose[3];
		// 正负限位
		double dLimit[12][2] = { 0 };
		dLimit[0][0] = config.jointSupremum[0];  dLimit[0][1] = config.jointInfimum[0];
		dLimit[1][0] = config.jointSupremum[1];  dLimit[1][1] = config.jointInfimum[1];
		dLimit[2][0] = config.jointSupremum[2];  dLimit[2][1] = config.jointInfimum[2];
		dLimit[3][0] = config.jointSupremum[3];  dLimit[3][1] = config.jointInfimum[3];
		dLimit[4][0] = config.jointSupremum[4];  dLimit[4][1] = config.jointInfimum[4];
		dLimit[5][0] = config.jointSupremum[5];  dLimit[5][1] = config.jointInfimum[5];
		for (int i = 6; i < 12; i++)
		{
			dLimit[i][0] = (i < 9) ? config.jointSupremum[i] : 3000;
			dLimit[i][1] = (i < 9) ? config.jointInfimum[i] : -3000;
		}
		// 关节轴最大角速度
		double JVelLimit[12] = { 0.0 };
		for (int i = 0; i < 12; ++i) {
			JVelLimit[i] = (i < 9) ? config.maxJointSpeedAuto[i] : 200;
		}
		m_Motion.SetInitPara(simStatus.m_timecycle, m_dDHPara, DGAxisCPos, ToolCPos, onlyCalcTime, dLimit, JVelLimit);

		return 0;
	}

} // namespace robot_interface
