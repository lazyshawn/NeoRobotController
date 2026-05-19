#pragma once

#include <QObject>
#include <QString>
#include <mutex>

#include "robot_interface/RobotTrajectory.h"

struct ModelData {
	// --- 1. 界面参数
	//! 当前界面显示和操作的机器人编号
	int robotIdx = 0;

	// --- 2. 主动切换的状态，需要发送到下位机
	//! 控制卡连接状态
	int cardConnectState = 0;
	//! 速度比例: [0,100]
	double speedRatio = 100;
	//! 点动坐标系
	int jogType = 0;
	//! 手自动模式
	bool autoMode = false;

	// --- 3. 轮询状态
	//! 上位机状态: [info, warning, error]
	int upperState[3];
	//! 下位机状态: [info, warning, error]
	int lowerState[3];
	//! 关节位置
	std::vector<double> jPos;
	//! 空间位置
	std::vector<double> cPos;
};

class MainModel : public QObject
{
	Q_OBJECT
public:
    explicit MainModel(QObject *parent = nullptr);
	~MainModel();

	// --- 查询状态
	// 连接状态
	int connect(const std::string& addr);
	int disconnect();
	int get_connectState() const;
	// 速度比例
	int set_speedRatio(double ratio);
	double get_speedRatio() const;
	// 选定机器人
	int set_robotIdx(int idx);
	int get_robotIdx() const;
	// 手自动模式
	int set_autoMode(bool enable);
	int get_autoMode() const;

	// --- 轮询状态
	// 返回模型状态快照，触发时更新状态
	void get_modelData(ModelData& data);

	// --- 对外接口
	// 设置点动类型
	int set_jog_type(int type);
	// 点动
	int jog_move(int robotIdx, int axisIdx, int dir, int enable);
	// 下发自动任务
	int push_trajectory(const FSAIRobotInterface::DiscreteTrajectory& trajectory);

	// 仿真线程: 后续替换为实际插补线程
	void sim_thread();

private:
	// mutable 允许在 const 成员函数中加锁, 考虑移动到 ModelData 中
	mutable std::mutex mtxModel;
	ModelData m_modelData;
};
