#pragma once


#include "robot_interface/FSAIRobot.h"


namespace FSAIRobotInterface {
// 仿真器状态
struct SimulatorStatus {
	// 插补周期
	double m_timecycle;

	bool autoMode = true;
	std::vector<double> jPos = std::vector<double>(12, 0);
	std::vector<double> cPos = std::vector<double>(12 ,0);
};

class FSAISimRobot : public RobotBase {

	// 接口
public:
	//! 获取下发指令轴号，主要用于确定运动主轴
	std::vector<int> get_execute_axis() override;

	// 机器人状态
	int update_rt_robot_status() override;
	int get_all_robot_status(RobotStatus& status) override;
	int get_register_config(RobotConfig& config);
	int read_register_config();
	int write_register_config(const RobotConfig& config);

	int moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask = {}) override;
	int moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask = {}) override;

	int moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask = {}) override;
	int moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask = {}) override;

	int moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask = {}) override;
	int moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask = {}) override;

	int move_compensate(const std::vector<float>& det) override;

	/**
	* @brief  设置手动速度比率
	* @param  ratio    速度比率(0-100)
	* @return 设置状态: 0 - 设置成功; 1 - 未处于手动模式
	*/
	int set_manual_speed(float ratio) override;

	// 自动任务
	int update_swing_config() override;
	int update_track_config() override;
	int update_welder_config() override;
	int get_remain_buffer() override;

	int push_new_trajectory(DiscreteTrajectory trajList) override;
	int execute_single_joint() override;
	int execute_single_cartesian() override;

	// 设置运动行号
	int send_line_num(int axis, const SingleTrajectory &curTraj) override;

	// 剩余缓冲检测
	int remain_buffer_free() override;

	// 一致性轨迹预处理，可以连续下发的轨迹
	int set_ready_for_consistent_traj(int& state) override;

	// 一致性轨迹就绪
	int consistent_traj_ready(int& state) override;

	int separate_trajectory() override;

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief  读取保存点位
	*/
	int read_saved_status(RobotStatus& status) override;

	int get_local_world_dpos(std::vector<float>& dpos) override;
	int cpos_base_to_world(std::vector<float>& cPos) override;

	int switch_auto(bool enableAuto) override;
	int switch_enable(bool enable) override;

	int reset_line_num() override;
	int set_jog_type(int type) override;
	int jog_moving(int type, int idx, int dir, int move) override;
	int save_task_status(bool enable, int inBuffer) override;

	int task_pause() override;
	int task_resume() override;
	int task_stop() override;
	int emergency_stop() override;

	// 设备操作
	int device_operation() override;

	int execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag) override;
	int process_after_send_traj() override;

	// 仿真器初始化: 切换运动仿真/计算时间
	int switch_robot_mode(int type) override;

private:
	RegisterBuffer begRegister;
	RegisterBuffer endRegister;

	/* *************************** 获取运动轴号 *************************** */
	inline int get_cmd_idx_base() {
		return 130000;
	}
	inline int get_point_idx_base() {
		return 160000;
	}

	// 设置上条轨迹类型
	int set_previous_trajectory(const SingleTrajectory& preTraj);

	SimulatorStatus simStatus;
	// 仿真器线程状态
	std::atomic<bool> simThreadDone;
	// 仿真器线程
	void sim_thread();

public:
	/* *************************** 初始化 *************************** */
	FSAISimRobot();
	~FSAISimRobot();

	// 写入缓冲寄存器
	int write_buffer_register(int flag);
};

} // namespace FSAIRobotInterface
