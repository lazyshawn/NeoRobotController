#pragma once

#include <memory>
#include <mutex>
#include <condition_variable>
#include <unordered_set>
#include <atomic>

#include "robot_interface/CoopRobotBase.h"
#include "robot_interface/NeoRobot.h"
#include "robot_interface/RobotTrajectory.h"
#include "robot_interface/BufferSynchronizer.h"

namespace FSAIRobotInterface {


/**
 * 机器人管理类
 *
 * 处理多机器人的状态更新，指令下发，任务协同等
 */
class RobotGroupManager {
	//! 协同就绪状态: 0 未就绪, 1 已就绪
	std::vector<int> syncReadyState;
	//! 协同就绪状态: <<type, num>, ...>
	//std::vector<Sync_Config> syncState;
	//! 机器人等待状态: <bit> <<robot, num>, ...>
	std::vector<std::unordered_set<int>> waitState;

	//! 线程终止条件
	bool workerHealthy = true;
	//! 指令处理线程, 状态更新线程, 下位机缓存读取线程
	std::thread cmdThreadWorker, updateThreadWorker, bufferThreadWorker;
	//! 指令线程状态
	std::atomic<bool> cmdThreadDone, bufferThreadDone;
	//! 保存机器人状态
	std::vector<RobotStatus> statusList;
	//! RobotGroupManager 状态
	std::vector<int> coopState;
	//! 已发送的轨迹，运动完成后的处理
	//std::vector<std::list<SingleTrajectory>> trajHistory;
	//! 机器人分组
	std::vector<std::vector<int>> disableGroup;
	//! 共用轴
	std::pair<int, int> sharedAxisState;


	/**
	* @brief  指令处理线程
	*/
	void processCommandThread();
	/** 
	* @brief  状态更新线程
	*/
	void updateStatusThread();
	/**
	* @brief  下位机缓存读取线程
	*/
	void readSlaveBufferThread();

	void set_group_sync_config(int robotIdx);

	/**
	* @brief  机器人同步就绪
	*/
	void update_sync_state(int robotIdx);

	/**
	* @brief  机器人到位处理
	*/
	void robot_in_place_command(int robotIdx);

	/**
	* @brief  关联机器人暂停
	*/
	int pause_coop_robot(int idx);

	/**
	* @brief  查询机器人组是否处于空闲状态
	*/
	bool robot_group_idle(const std::vector<int>& ids = {});
	/**
	* @brief  查询机器人是否协同就绪
	*/
	bool robot_sync_ready(int robotIdx);

	// 计算协同段总运动时间
	int calc_sync_duration(int robotIdx);

	// 修正协同段轨迹速度
	void correct_sync_speed();

	// IO 等待标志复位
	void reset_wait_state(int robotIdx);


public:
	//! 机器人队列
	std::vector<std::shared_ptr<RobotBase>> robotList;

	RobotGroupManager();
	~RobotGroupManager();

	/**
	* @brief  向指定控制卡申请新机器人
	* @param  robot    机器人类指针
	* @return 机器人 ID
				   < 0    机器人创建失败
				   >=0    机器人 ID
	*/
	int new_robot(std::shared_ptr<RobotBase> robot);

	/**
	* @brief  设置共用轴
	* @param  controllerID    控制卡 ID
	*/
	int set_shared_axis(int axisId, const std::vector<int>& robotId);

	/**
	* @brief  开启线程，开始管理机器人组状态
	*/
	int start_thread();

	/**
	* @brief  结束线程
	*/
	int stop();

	/**
	* @brief  查询机器人是否处于错误状态
	*/
	bool robot_error(int idx);
	/**
	* @brief  查询机器人是否处于警告状态
	*/
	bool robot_warning(int idx);
	/**
	* @brief  查询机器人是否处于空闲状态
	*/
	bool robot_idle(int idx);

	/**
	* @brief  机器人组继续
	*/
	int robot_group_resume(int idx);
	int robot_group_resume(const std::vector<int>& idxList);
	/**
	* @brief  机器人组暂停
	*/
	int robot_group_pause(int idx);
	int robot_group_pause(const std::vector<int>& idxList);
	/**
	* @brief  机器人组暂停后更新位置
	*/
	int robot_group_update_saved_pos(const std::vector<int>& idxList);
	/**
	* @brief  机器人组清空任务
	*/
	int robot_group_clear_task(int idx);
	/**
	* @brief  机器人组急停
	*/
	int robot_group_stop(int idx);

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
