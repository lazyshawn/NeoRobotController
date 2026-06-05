#pragma once

#include <mutex>
#include <condition_variable>
#include <queue>

#include "common/ExportSharedAPI.h"

#include "robot_interface/RobotTrajectory.h"

namespace FSAIRobotInterface {

/**
* 机器人实时状态参数
*
* 需要高频更新的实时参数
* 每次轮询都更新一次，以保证机器人管理线程的正常运行
*/
struct RobotStatusBuffer {
	std::vector<float> slaveBuffer;
};


/**
* 机器人接口基类
*
* 与底层控制卡绑定的交互接口，实现机器人运控算法与流程的解耦
* 只需要定义管理类`RobotGroupManager`的必要接口
* Example:
*   class Derived : public RobotBase {
*     \\ foo ...
*   }
*   std::shared_ptr<RobotBase> robot(new Derived);
*   RobotGroupManager group;
*   group.new_robot(robot);
*/
class RobotBase {
	//! 专属管理类
	friend class RobotGroupManager;

protected:
	//! 控制卡分配的 ID
	int robotId = -1;
	//! 指定编号
	int aliasId = -1;

	//! 控制卡
	//std::shared_ptr<Controller> ZController;
	//! 屏蔽轴号
	//std::unordered_set<int> axisMask;

	// 互斥锁与条件变量
	std::mutex mtxMotion, mtxInnerBuffer, mtxOuterBuffer;
	std::condition_variable cvMotion;
	bool motionDone = false;

	//! 配置参数
	RobotConfig robotConfig;
	//! 机器人状态
	RobotStatus robotStatus;

	//! 机器人缓存轨迹
	DiscreteTrajectory trajectory;
	//! 已发送的轨迹，运动完成后的处理
	DiscreteTrajectory trajHistory;

public:
	RobotBase();
	virtual ~RobotBase();

	/* *******************************************************
	* 统一接口
	* ***************************************************** */
	//! 等待机器人运动停止
	int wait_auto_task_stop();
	//! 获取当前保存的机器人状态
	int get_rt_robot_status(RobotStatus& status);

	/* *******************************************************
	* 虚函数接口
	* ***************************************************** */

private:
	/* *******************************************************
	* 对内接口
	*
	* 机器人管理类需要封装的接口，具体实现与机器人类无关
	* ***************************************************** */
	// --- 机器人管理类相关
	//! 唤醒等待中的线程
	int notify_waiting_robot();
	//! 已下发任务完成
	int task_assigned_completed();

	//! 设置上位机状态码
	int set_upperStatus(int idx, int code);
	//！ 恢复上位机状态码
	int reset_upperStatus(int idx = -1);

	/**
	* @brief  补充缓冲运动参数
	*
	* 指令下发前，将配置参数封装到对应的缓冲动作中
	*/
	int rewrie_actioin_param(MoveActionConfig& cfg);

	/**
	* @brief  插入任务轨迹
	*/
	int insert_task_traj();

	// 起弧成功检测
	bool check_arc_on();

	// --- 控制卡相关
	//! 获取保存的机器人配置参数
	int get_register_config(RobotConfig& config);

	//! 读取VR寄存器中的配置参数
	int read_register_config();
	//! 更新VR寄存器中的配置参数
	int write_register_config(const RobotConfig& config);

	//! 捕获日志
	int capture_controller_log();

	//! 下发在线指令
	int send_command(const std::string& cmd, std::string& ack, int type);
	//! 机器人程序重启
	int reboot(const char *basPath, int mode);
	//! 加载所有配置
	int load_config(const std::string& fname);
	//! 导出所有配置
	int export_config(const std::string& fname);

	/**
	* @brief  触发底层封装好的运动指令
	* @param    type      运动类型
	* @param    action    运动参数
	*/
	int trigger_action(int type, const std::vector<double>& param);

	//! 获取自定义的下位机缓存数据: 如电弧跟踪、视觉伺服数据
	int get_slave_buffer();

	/**
	* @brief  上位机与下位机缓冲数据同步
	* @param  masterStamp    同步时刻上位机时间戳
	* @param  slaveStamp     同步时刻下位机时间戳
	*/
	int synchronize_slave_buffer(uint64_t& masterStamp, uint64_t& slaveStamp);

	/**
	* @brief  查询下位机缓冲数据
	* @param  [out]  buffer   查询结果
	* @param         popFlag  清空已查询数据
	* @param         num      查询个数, <= 0: 全部读取
	*/
	//int pop_slave_buffer(std::vector<motion::BufferUnit>& buffer, bool popFlag, int num = -1);

	/**
	* @brief  查找并补偿轨迹缓冲中的目标点
	* @param  id    下发轨迹时设定的 rewriteId
	* @param  pos   补偿点的目标位置
	* @return   0 - 正常返回
				1 - 未找到对应id的点位
				2 - 修正量过大警告
	*/
	int modify_point_in_buffer(int id, const std::vector<double>& pos);

	/* *******************************************************
	* 对内纯虚接口
	*
	* 机器人管理类需要封装的接口，每个机器人类不同
	* ***************************************************** */
	/**
	* @brief  缓冲中执行底层封装好的运动指令
	* @param    action    运动参数
	* @param    flag      手动标识运动前/后动作
	*/
	virtual int execute_move_action(const std::vector<std::pair<int, std::vector<double>>>& actionList, int flag = 0);

	// 轨迹下发后处理
	virtual int process_after_send_traj();

	virtual int execute_single_joint() = 0;
	virtual int execute_single_cartesian() = 0;

	//! 剩余缓冲检测
	virtual int remain_buffer_free() = 0;

	//! 读取断点信息
	virtual int read_saved_status(RobotStatus& status) = 0;

	/**
	* @brief  一致性轨迹预处理
	* @param  state  机器人组状态
	*
	* 不同算法，可能轨迹段开始前需要预存数据，或其他预处理
	* 该功能用于检测是否可以进行预处理，并修改下发标识，让轨迹可以通过后续的一致性检测
	*/
	virtual int set_ready_for_consistent_traj(int& state) = 0;

	/**
	* @brief  一致性轨迹就绪检测
	* @param  state  机器人组状态
	*
	* 检测是否当前轨迹可以开始下发，若不能则判断并执行切换动作，等待下次判断
	* 该功能可以阻止未进行预处理的轨迹被下发
	*/
	virtual int consistent_traj_ready(int& state) = 0;

	virtual int separate_trajectory() = 0;

	/* *******************************************************
	* 通用接口
	*
	* 机器人管理类与上位机交互的接口，具体实现与机器人类无关
	* ***************************************************** */
	//! 机器人设置初始化: 仿真/真机
	virtual int switch_robot_mode(int type);

	/* *******************************************************
	* 通用纯虚接口
	*
	* 机器人管理类与上位机交互的接口，每个机器人类不同
	* ***************************************************** */
	/*
	* 从控制器或仿真器更新当前机器人状态
	*
	* 只能在一处调用(CoopRobotManager)，其他线程通过 get_rt_robot_status 同步状态
	*/
	virtual int update_rt_robot_status() = 0;

	//! 下发运动补偿
	virtual int move_compensate(const std::vector<float>& det) = 0;
	//! 切换使能
	virtual int switch_enable(bool enable) = 0;
	//! 修改速度比例
	virtual int set_manual_speed(float ratio) = 0;
	//! 切换手自动模式
	virtual int switch_auto(bool enableAuto) = 0;
	//! 下发自动任务
	virtual int push_new_trajectory(DiscreteTrajectory trajList) = 0;

	/**
	* @brief  设置点动类型
	* @param  type
	*      -# 0: 关节
	*      -# 1: 世界坐标系
	*      -# 2: 工具坐标系
	*/
	virtual int set_jog_type(int type) = 0;
	/**
	* @brief  Jog 点动
	* @param  type    运动类型
			-# 0: 关节运动
			-# 1: 世界坐标系运动
			-# 2: 工具坐标系运动
	* @param  idx     运动轴号
					0,   1,   2,   3,   4,   5,   6,   7,   8
			关节    J1,  J2,  J3,  J4,  J5,  J6,  G1,  G2,  G3
			世界    x,   y,   z,   Rx,  Ry,  Rz,  G1,  G2,  G3
			机器人  x,   y,   z,   Rx,  Ry,  Rz,  G1,  G2,  G3
			工具    x,   y,   z,   Rx,  Ry,  Rz,  G1,  G2,  G3
	* @param  dir    区分运动方向
			-# 0 : 停止运动
			-# 1 : 正向运动
			-# -1: 负向运动
	* @param  move   区分是否运动, 部分算法可能需要分别指定方向和是否运动
			-# 0: 停止运动
			-# 1: 开始运动
	*/
	virtual int jog_moving(int type, int idx, int dir, int move) = 0;

	virtual int task_pause() = 0;
	virtual int task_resume() = 0;
	virtual int task_clear() = 0;
	virtual int emergency_stop() = 0;
};

}
