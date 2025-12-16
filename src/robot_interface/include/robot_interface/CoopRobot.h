#pragma once

#include <memory>
#include <mutex>
#include <condition_variable>
#include <unordered_set>
#include <atomic>

#include "ZMotionController.h"
#include "RobotTrajectory.h"
#include "ParamSerialization.h"
#include "BufferSynchronizer.h"

//#include "BaseDef.h"

namespace FSAIRobotInterface {

/**
 * 机器人配置参数
 */
struct RobotConfig {
	// 机器人配置参数
	// 连杆参数: LargeZ,L1,L2,L3,L4,D5,DiffY
	std::vector<float> linkLength = {};
	// 编码器位数
	std::vector<float> encoderBit = {};
	// 轴电机减速比/传动比(更新地轨)
	//std::vector<float> transRatio = {};
	// 减速比分子
	std::vector<float> transRatioNumerator = {};
	// 减速比分母
	std::vector<float> transRatioDenominator = {};
	// 耦合比
	std::vector<float> couplingConfig = {};
	// TCP 参数: SmalLX,SmalLY,SmalLZ,InitRx,InitRy,InitRz(更新tcp)
	std::vector<float> eefPose = {}, tcpPose = {};
	// 关节上限位(更改)
	std::vector<float> jointSupremum = {};
	// 关节下限位(更改)
	std::vector<float> jointInfimum = {};
	// 关节自动模式最大速度
	std::vector<float> maxJointSpeedAuto = {};
	// 关节手动模式最大速度
	std::vector<float> maxJointSpeedManual = {};
	// 末端手动模式最大速度
	std::vector<float> maxCartSpeedManual = {};
	// IO 配置(更改)
	std::vector<int> ioAction = {};
	// 附加轴标定结果(更改)
	std::vector<float> auxCalbration = {};
	// 主从机标定结果(更改)
	std::vector<float> slaveCalibration = {};
	// 零点编码器值(更改)
	std::vector<float> zeroEncoder = {};
	// 从属设备ID(更改)
	std::vector<int> slaveDeviceID = {};
	//! 从属设备类型
	std::vector<int> slaveDeviceType = {};

	// 附加轴轴号
	std::vector<int> appAxisIdx;
	std::vector<int> appAxisIdxRead;

	RobotConfig() {};
	~RobotConfig() {};

	/* *************************** 配置接口 *************************** */
	/**
	* @brief IO 配置
	* @param    index    IO输入口
	* @param    lowerStatus   有效状态
	*               -1   屏蔽
	*                0   低电平有效
	*                0   高电平有效
	* @param    action   触发动作
	*                1   机器人暂停
	*                2   机器人急停
	*                3   运动中暂停，否则急停
	* @param
	*/
	int config_input_action(const std::vector<int>& index, const std::vector<int>& lowerStatus, const std::vector<int>& action);
	/**
	* @brief 零点设置
	* @param zeroEncoder
	*/
	int config_zero_point();
	/**
	* @brief 原点设置
	* @param homeEncoder
	*/
	int config_home_point();
	/**
	* @brief TCP 姿态设置
	* @param tcpIdx     TCP 序号
	* @param tcpPose    TCP 位姿
	*/
	int config_tcp_pose();

	// 连杆参数
	int convert_dhParam_to_linkLength();

	Eigen::Matrix3f get_slave_calibratino_mat();
};


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
 * 机器人状态参数
 *
 * 当前所有参数都在该类中，后续实时参数将移动到 `RobotRTStatus` 类中
 */
struct RobotStatus {
	// 实时刷新
	int lowerStatus;                      // 下位机状态(Bit)：(0)
	int upperStatus = 0;                  // 上位机状态: 指令下发异常，手动/自动模式不匹配

	int autoMode;                         // 自动模式：  -1-手动模式，1-自动模式
	int fkMode;                           // 正逆解模式: 0-未建立，1-正解, -1-逆解
	int lineNum = 0;                      // 当前运动行号
	int cmdNum = 0;                       // 当前轨迹的下发编号
	float masterAxisDist;                 // 主轴运动距离
	int remainBuffer;					  // 剩余缓冲数

	float current;                        // 实时电流
	float voltage;                        // 实时电压

	long slaveTime;                       // 下位机时间戳
	long weldTime;                        // 焊接时间
	long weldBegTime;                     // 上次起弧时间戳
	long weldEndTime;                     // 上次息弧时间戳

	std::vector<float> jPos = {};         // 关节位置
	std::vector<float> cPos = {};         // 上位机的笛卡尔空间位置
	std::vector<float> cPosRaw = {};      // 控制卡中的笛卡尔空间位置
	std::vector<float> cPosBuffer = {};   // 缓冲中目标位置
	//int taskId;                           // 当前任务号
	//int taskType;                         // 当前任务类型：空移，拍照，横焊，立焊，平焊

	// 按需刷新 (非实时)
	std::vector<int> axisStatus = {};     // 轴状态
	std::vector<int> encoder = {};        // 编码器值
	std::vector<float> posOffset = {};    // 随动偏移
	std::vector<float> cPosR = {};        // 机器人坐标系位置
	std::vector<int> subErrorCode = {}; // 异常码辅码

	RobotStatus() {};
	~RobotStatus() {};
};


/**
 * 机器人接口基类
 * 
 * 与底层控制卡绑定的交互接口，实现机器人运控算法与流程的解耦
 * Example:
 *   class Derived : public RobotBase {
 *     \\ foo ...
 *   }
 *   std::shared_ptr<RobotBase> robot(new Derived);
 *   RobotGroupManager group;
 *   group.new_robot(robot);
 */
class RobotBase {
protected:
	//! 控制卡分配的 ID
	int robotId = -1;
	//! 指令行号: 下发的运动个数
	int cmdNum = 0;
	//! 指定编号
	int aliasId = -1;
	//! 状态刷新线程
	bool enableRefresh = false;

	//! 控制卡
	std::shared_ptr<Controller> ZController;
	//! 屏蔽轴号
	std::unordered_set<int> axisMask;

	// 互斥锁与条件变量 
	std::mutex mtxMotion, mtxBuffer;
	std::condition_variable cvMotion;
	bool motionDone = false;

	//! 配置参数
	RobotConfig robotConfig;
	//! 机器人状态
	RobotStatus robotStatus;
	//! 状态缓存
	//RobotStatusBuffer statusBuffer;
	//！ 跟踪数据
	BufferSynchronizer bufferSync;

public:
	virtual ~RobotBase();

	// 机器人缓存轨迹
	DiscreteTrajectory trajectory;

	/* *************************** 通用接口 *************************** */
	//! 关节起始编号
	inline int get_joint_idx_base() {
		return 32 * robotId;
	}
	//! 配置参数起始编号
	inline int get_config_idx_base() {
		return 1000 * (robotId + 1);
	}
	//! 状态参数起始编号
	inline int get_state_idx_base() {
		return 5000 + 2000 * (robotId + 1);
	}
	//! 数据参数起始编号
	inline int get_data_idx_base() {
		return 20000 + 20000 * (robotId + 1);
	}
	//! 获取机器人 ID
	inline int get_robotId() {
		return robotId;
	}
	//! 设置机器人别称 ID
	inline void set_aliasId(int id) {
		aliasId = id;
	}
	//! 获取机器人别称 ID
	inline int get_aliasId() {
		return aliasId;
	}
	//! 获取下发轨迹编号
	inline int get_lineNum() {
		return cmdNum;
	}
	std::vector<int> get_joint_axis();
	std::vector<int> get_axis_idx();

	//! 唤醒等待中的线程
	int notify_waiting_robot();
	//! 等待机器人运动停止
	int wait_auto_task_stop();


	// 设置控制器句柄
	int set_ZController(std::shared_ptr<Controller> ZController_, int id = -1);
	// 组合轴号
	std::vector<int> get_composed_axis(const std::vector<std::vector<int>>& axisList);
	//! 获取保存的机器人状态
	int get_rt_robot_status(RobotStatus& status);
	//! 获取保存的机器人配置参数
	int get_register_config(RobotConfig& config);
	/**
	* @brief 读取VR寄存器中的配置参数
	*/
	int read_register_config();
	/**
	* @brief 更新VR寄存器中的配置参数
	*/
	int write_register_config(const RobotConfig& config);
	/**
	* @brief 屏蔽共用附加轴
	* @param axis       屏蔽轴号
	*/
	int set_axisIdxMask(const std::vector<int>& axis);
	//! 查找轨迹指令是否包含指定轴号
	int find_command_axis(const SingleTrajectory& traj, int axis);
	
	//! 设置上位机状态码
	int set_upperStatus(int code);
	//！ 恢复上位机状态码
	int reset_upperStatus(int idx = -1);

	//! 捕获日志
	int capture_controller_log();

	//! 下发指令
	int send_command(const std::string& cmd, std::string& ack, int type);

	int reboot(const char *basPath,  int mode);
	int load_config(const std::string& fname);
	int export_config(const std::string& fname);

	/**
	* @brief  触发底层封装好的运动指令
	* @param    type      运动类型
	* @param    action    运动参数
	*/
	int trigger_action(int type, const std::vector<float>& param);

	/**
	* @brief  导出电弧跟踪数据
	*/
	int export_tracking_data();

	// IO 有效状态
	int get_input_effective_state(const std::vector<int>& ioNum, std::vector<int>& state);
	int set_input_effective_state(const std::vector<int>& ioNum, const std::vector<int>& state);

	// IO 触发动作
	int get_input_action(const std::vector<int>& ioNum, std::vector<int>& action);
	int set_input_action(const std::vector<int>& ioNum, const std::vector<int>& action);

	int get_input(int ioNum);
	int get_input_invert(int ioNum);
	int set_input_invert(int ioNum, int state);
	int get_output(int ioNum);
	int set_output(int ioNum, int state);
	int send_welding_wire(int dir);
	int welder_blow(bool on);

	int reset_dual_axis_home_position();
	int read_action_result(std::vector<float>& result);
	int get_multilayer_pos(std::vector<float>& pos);

	// 开启缓存读取线程
	int slave_buffer_stream(bool enable);
	// 获取自定义的下位机缓存数据: 如电弧跟踪、激光跟踪数据
	int get_slave_buffer();

	/**
	* @brief  单轴使能
	* @param  enable    使能标志
	* @param  axis      >=0   使能轴号
	*                   < 0   所有轴号
	*/
	int single_axis_enable(bool enable, int axis = -1);

	/**
	* @brief  上位机与下位机缓冲数据同步
	*/
	int synchronize_slave_buffer(long long masterStamp);

	/**
	* @brief  查询下位机缓冲数据
	*/
	int query_slave_buffer(long long stamp, std::vector<float>& data);

	/* *************************** 底层可修改接口 *************************** */
	/**
	* @brief  缓冲中执行底层封装好的运动指令
	* @param    type      运动类型
	* @param    action    运动参数
	*/
	virtual int execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag = 0);

	// 轨迹下发后处理
	virtual int process_after_send_traj();

	// 设置当前轨迹类型
	virtual int send_traj_type(int type);


	/* *************************** 底层自定义接口 *************************** */

	//! 获取下发指令轴号，主要用于确定运动主轴
	virtual std::vector<int> get_execute_axis() = 0;

	// 机器人状态
	virtual int update_rt_robot_status() = 0;
	virtual int get_all_robot_status(RobotStatus& status) = 0;

	virtual int moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) = 0;
	virtual int moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) = 0;

	virtual int moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) = 0;
	virtual int moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) = 0;

	virtual int moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) = 0;
	virtual int moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) = 0;

	virtual int set_manual_speed(float ratio) = 0;

	// 自动任务
	virtual int update_swing_config() = 0;
	virtual int update_track_config() = 0;
	virtual int update_welder_config() = 0;
	virtual int get_remain_buffer() = 0;

	virtual int push_new_trajectory(DiscreteTrajectory trajList) = 0;
	virtual int execute_single_joint() = 0;
	virtual int execute_single_cartesian() = 0;


	// 下发轨迹编号，运动完修改
	virtual int send_line_num(int axis, const SingleTrajectory &curTraj) = 0;

	// 剩余缓冲检测
	virtual int remain_buffer_free() = 0;

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

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief 读取VR寄存器中的配置参数
	*/
	//virtual int read_register_config() = 0;
	/**
	* @brief 更新VR寄存器中的配置参数
	*/
	//virtual int write_register_config(const RobotConfig& config) = 0;
	virtual int read_saved_status(RobotStatus& status) = 0;

	virtual int get_local_world_dpos(std::vector<float>& dpos) = 0;
	virtual int cpos_base_to_world(std::vector<float>& cPos) = 0;

	virtual int switch_auto(bool enableAuto) = 0;
	virtual int switch_enable(bool enable) = 0;

	virtual int reset_line_num() = 0;
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
		   -# 2: 工具坐标系运动 (todo)
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

	virtual int save_task_status(bool enable, int inBuffer) = 0;

	virtual int task_pause() = 0;
	virtual int task_resume() = 0;
	virtual int task_stop() = 0;
	virtual int emergency_stop() = 0;

	// 设备操作
	virtual int device_operation() = 0;

};


/**
 * 机器人管理类
 *
 * 处理多机器人的状态更新，指令下发，任务协同等
 */
class RobotGroupManager {
	//! 协同就绪状态: 0 未就绪, 1 已就绪
	std::vector<int> syncReadyState;
	//! 协同就绪状态: <<type, num>, ...>
	std::vector<Sync_Config> syncState;
	//! 机器人等待状态: <bit> <<robot, num>, ...>
	std::vector<std::unordered_set<int>> waitState;

	//! 线程终止条件
	bool workerHealthy = true;
	//! 指令处理线程, 状态更新线程
	std::thread cmdThreadWorker, updateThreadWorker;
	//! 指令线程状态
	std::atomic<bool> cmdThreadDone;
	//! 保存机器人状态
	std::vector<RobotStatus> statusList;
	//! RobotGroupManager 状态
	std::vector<int> coopState;
	//! 已发送的轨迹，运动完成后的处理
	std::vector<std::list<SingleTrajectory>> trajHistory;
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
