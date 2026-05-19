#pragma once

#include <memory>

#include "robot_interface/RobotTrajectory.h"

namespace FSAIRobotInterface {

/**
 * 机器人管理类
 *
 * 处理多机器人的状态更新，指令下发，任务协同等
 */
class SHARE_API_ RobotGroupManager {
	struct IMPL;
	std::unique_ptr<IMPL> pimpl;

public:
	RobotGroupManager();
	~RobotGroupManager();
	// 禁用拷贝
	RobotGroupManager(const RobotGroupManager&) = delete;
	RobotGroupManager& operator=(const RobotGroupManager&) = delete;
	// 禁用移动
	RobotGroupManager(RobotGroupManager&& other) = delete;
	RobotGroupManager& operator=(RobotGroupManager&& other) = delete;

	// 机器人类型
	enum { NEOROBOT };

	/**
	* @brief  向指定控制卡申请新机器人
	* @param  type    机器人类型
	*/
	int new_robot(int type);

	int get_rt_robot_status(int idx, RobotStatus& robotStatus);
	int push_new_trajectory(int idx, const DiscreteTrajectory& trajectory);
	int switch_auto(int idx, bool enable);
	int jog_moving(int robotIdx, int type, int idx, int dir, int move);

	/**
	* @brief  开启线程，开始管理机器人组状态
	*/
	int start_thread();

	/**
	* @brief  结束线程
	*/
	int stop();

	// 开启缓存读取线程
	int slave_buffer_stream(bool enable);
};

/**
 * @brief  设置bit位
 * @param[out]  state    当前状态
 * @param       idx      待修改 bit 位
 * @param       enable   写入状态
 */
int set_bit(int& state, int idx, bool enable);

/**
 * @brief  获取bit位
 */
int get_bit(int state, int idx);

} //namespace FSAIRobotInterface
