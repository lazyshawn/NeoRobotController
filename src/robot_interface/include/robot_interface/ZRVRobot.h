#pragma once
/*********************************
 * TODO:
 * 1. 轴号屏蔽。
 * 1. 暂停后点动。
 *********************************/

#include "CoopRobot.h"


namespace FSAIRobotInterface {

class ZRVRobot : public RobotBase {

	// 接口
public:
	//! 获取下发指令轴号，主要用于确定运动主轴
	std::vector<int> get_execute_axis() override;

	// 机器人状态
	int update_rt_robot_status() override;
	int get_all_robot_status(RobotStatus& status) override;

	int moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) override;
	int moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) override;

	int moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) override;
	int moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) override;

	int moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) override;
	int moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) override;

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
	int set_ready_for_consistent_traj() override;

	// 一致性轨迹就绪
	int consistent_traj_ready(int& state) override;

	int separate_trajectory() override;

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief 读取VR寄存器中的配置参数
	*/
	int read_register_config() override;
	/**
	* @brief 更新VR寄存器中的配置参数
	*/
	int write_register_config(const RobotConfig& config) override;

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

	/* *************************** 可修改接口 *************************** */
	// 轨迹下发后处理
	int process_after_send_traj() override;

private:


public:
	ZRVRobot();
	~ZRVRobot();

	std::vector<int> get_cam_axis();
	std::vector<int> get_joint_axis();
	std::vector<int> get_tcp_axis();
	std::vector<int> get_robot_tcp_axis();


	// 摆焊参数
	int get_swing_num();

	int update_swing_table(const Weave& waveCfg);

	int swing_on(float dist, const Weave& waveCfg, int mode, const std::vector<float>& toolDir, const std::vector<float>& planeDir);

	int swing_off(float displacement);

	TrajectoryPoint partition_trajectory(const TrajectoryPoint& preTraj, const TrajectoryPoint& curTraj, DT_scale begRatio, DT_scale endRatio, int mode);

	int switch_kinematics(int mode, int retry = 10);

};

} // namespace FSAIRobotInterface
