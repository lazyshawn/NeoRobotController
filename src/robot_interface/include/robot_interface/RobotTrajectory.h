#pragma once

#include <deque>

#include "common/TrajectorySegment.h"
#include "common/ExportSharedAPI.h"


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
	// 关节自动模式最大加速度
	std::vector<float> maxJointAccAuto = {};
	// 关节手动模式最大加速度
	std::vector<float> maxJointAccManual = {};
	// 末端自动模式最大速度
	std::vector<float> maxCartSpeedAuto = {};
	// 末端手动模式最大速度
	std::vector<float> maxCartSpeedManual = {};
	// 末端自动模式最大加速度
	std::vector<float> maxCartAccAuto = {};
	// 末端手动模式最大加速度
	std::vector<float> maxCartAccManual = {};

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

	//Eigen::Matrix3f get_slave_calibratino_mat();
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
	double masterAxisDist;                // 主轴运动距离
	int remainBuffer;					  // 剩余缓冲数

	double current;                        // 实时电流
	double voltage;                        // 实时电压

	long slaveTime;                       // 下位机时间戳
	long weldTime;                        // 焊接时间
	long weldBegTime;                     // 上次起弧时间戳
	long weldEndTime;                     // 上次息弧时间戳

	std::vector<double> jPos = {};         // 关节位置
	std::vector<double> cPos = {};         // 上位机的笛卡尔空间位置
	std::vector<double> cPosRaw = {};      // 控制卡中的笛卡尔空间位置
	std::vector<double> cPosBuffer = {};   // 缓冲中目标位置
	std::vector<double> jSpeed = {};       // 关节速度
	//int taskId;                           // 当前任务号
	//int taskType;                         // 当前任务类型：空移，拍照，横焊，立焊，平焊

	// 按需刷新 (非实时)
	double curInterpTime = 0.0;           // 当前插补时间(目前仅仿真器使用)
	std::vector<int> axisStatus = {};     // 轴状态
	std::vector<int> encoder = {};        // 编码器值
	std::vector<double> posOffset = {};    // 随动偏移
	std::vector<double> cPosR = {};        // 机器人坐标系位置
	std::vector<int> subErrorCode = {};   // 异常码辅码

	RobotStatus() {};
	~RobotStatus() {};
};


class SHARE_API_ SingleTrajectory : public SegmentBase {
public:
	bool isJoint();
	bool isCartesian();
};

// 上位机任务的离散轨迹指令
class SHARE_API_ DiscreteTrajectory {
	std::deque<SingleTrajectory> trajList;
	SingleTrajectory preTraj;

public:
	//! 清空轨迹
	int clear();
	//! 添加轨迹
	int add_single_traj(const SingleTrajectory& traj);
	int push_trajectory(const DiscreteTrajectory& trajectory);
	//! 弹出轨迹
	int pop();

	bool trajectory_loaded();
	SingleTrajectory get_curTraj();
	SingleTrajectory get_preTraj();
};

} // namespace FSAIRobotInterface
