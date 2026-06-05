#pragma once

#include "CoopRobotBase.h"

namespace FSAIRobotInterface {

class NeoRobot : public RobotBase {
	// --- 插补线程相关
	//! 线程终止条件
	bool interpWorkerHealthy = true;
	//! 插补线程
	std::thread interpThreadWorker;
	//! 插补线程接口
	void interp_thread();

public:
	explicit NeoRobot();
	virtual ~NeoRobot();

	// --- 内部接口
	//! 执行缓冲运动
	int execute_move_action(const std::vector<std::pair<int, std::vector<double>>>& actionList, int flag) override;

	//! 轨迹下发后处理
	int process_after_send_traj() override;

	int execute_single_traj(SingleTrajectory& outTraj) override;

	//! 剩余缓冲检测
	int remain_buffer_free() override;

	//! 读取断点信息
	int read_saved_status(RobotStatus& status) override;

	int set_ready_for_consistent_traj(int& state) override;

	int consistent_traj_ready(int& state) override;

	int separate_trajectory() override;

	int update_rt_robot_status() override;


	// --- 对外接口
	//! 下发运动补偿
	int move_compensate(const std::vector<float>& det) override;

	//! 下发自动任务
	int push_new_trajectory(DiscreteTrajectory trajList) override;

	int switch_auto(bool enableAuto) override;
	int switch_enable(bool enable) override;
	int set_manual_speed(float ratio) override;

	//! 设置点动类型
	int set_jog_type(int type) override;
	//! 点动执行
	int jog_moving(int type, int idx, int dir, int move) override;

	//! 任务暂停
	int task_pause() override;
	//! 任务继续
	int task_resume() override;
	//! 任务清空
	int task_clear() override;
	//! 急停
	int emergency_stop() override;
};

}
