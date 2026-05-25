#include "MainModel.h"

#include "robot_interface/CoopRobotManager.h"

static FSAIRobotInterface::RobotGroupManager group;

MainModel::MainModel(QObject *parent) : QObject(parent) {
	// 启动机器人管理类
	group.new_robot(FSAIRobotInterface::RobotGroupManager::NEOROBOT);
	group.start_thread();
}

MainModel::~MainModel() {
	// 结束线程
	group.stop();
}

int MainModel::connect(const std::string& addr) {
	// 连接控制卡
	// 检测连接状态
	// 更新连接状态，控制卡地址
	m_modelData.cardConnectState = true;
	return 0;
}

int MainModel::disconnect() {
	m_modelData.cardConnectState = false;
	return 0;
}

int MainModel::get_connectState() const {
	std::lock_guard<std::mutex> lock(mtxModel);
	return m_modelData.cardConnectState;
}

int MainModel::set_speedRatio(double ratio) {
	std::lock_guard<std::mutex> lock(mtxModel);
	m_modelData.speedRatio = ratio;
	return 0;
}

double MainModel::get_speedRatio() const {
	std::lock_guard<std::mutex> lock(mtxModel);
	return m_modelData.speedRatio;
}

int MainModel::set_robotIdx(int idx) {
	std::lock_guard<std::mutex> lock(mtxModel);
	m_modelData.robotIdx = idx;
	return 0;
}

int MainModel::get_robotIdx() const {
	std::lock_guard<std::mutex> lock(mtxModel);
	return m_modelData.robotIdx;
}

int MainModel::set_autoMode(bool enable) {
	group.switch_auto(m_modelData.robotIdx, enable);
	return 0;
}

int MainModel::get_autoMode() const {
	return m_modelData.autoMode;
}

void MainModel::get_modelData(ModelData& data) {
	// 从轮询线程中更新类成员变量
	FSAIRobotInterface::RobotStatus robotStatus;
	group.get_rt_robot_status(0, robotStatus);

	std::lock_guard<std::mutex> lock(mtxModel);
	m_modelData.jPos = robotStatus.jPos;
	m_modelData.autoMode = robotStatus.autoMode;

	// 返回更新后的值
	data = m_modelData;
}

int MainModel::set_jogType(int robotIdx, int type) {
	group.set_jogType(robotIdx, type);
	return 0;
}

int MainModel::jog_move(int robotIdx, int axisIdx, int dir, int enable) {
	group.jog_moving(robotIdx, 0, axisIdx, dir, enable);
	return 0;
}

int MainModel::push_trajectory(const FSAIRobotInterface::DiscreteTrajectory& trajectory) {
	group.push_new_trajectory(0, trajectory);
	return 0;
}
