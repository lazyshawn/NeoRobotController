#include "MainModel.h"

#include "robot_interface/CoopRobotManager.h"

static std::shared_ptr<FSAIRobotInterface::RobotBase> robot(new FSAIRobotInterface::NeoRobot);
static FSAIRobotInterface::RobotGroupManager group;

MainModel::MainModel(QObject *parent) : QObject(parent) {
	bool simulate = false;

	if (simulate) {
		// 开启插补线程，更新机器人状态
		std::thread sim_thread(&MainModel::sim_thread, this);
		sim_thread.detach();
	}
	else {
		robot->switch_auto(true);
		group.new_robot(robot);
		group.start_thread();
	}
}

MainModel::~MainModel() {
	// 结束线程
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

void MainModel::get_modelData(ModelData& data) {

	// 从轮询线程中更新类成员变量
	FSAIRobotInterface::RobotStatus robotStatus;
	robot->get_rt_robot_status(robotStatus);

	std::lock_guard<std::mutex> lock(mtxModel);
	m_modelData.jPos = robotStatus.jPos;

	// 返回更新后的值
	data = m_modelData;
}

int MainModel::set_jog_type(int type) {
	return 0;
}

int MainModel::jog_move(int robotIdx, int axisIdx, int dir, int enable) {
	printf("jog: %d, %d, %d, %d\n", robotIdx, axisIdx, dir, enable);
	return 0;
}

int MainModel::push_trajectory(const DiscreteTrajectory& trajectory) {
	group.robotList[0]->push_new_trajectory(trajectory);
	return 0;
}

void MainModel::sim_thread() {
	static std::atomic<bool> simThreadDone;
	static int cnt = 0;

	simThreadDone.store(false);
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = static_cast<int>(50);

	while (!simThreadDone) {
		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		auto now = std::chrono::steady_clock::now();

		// 加锁，更新模型数据
		{
			std::lock_guard<std::mutex> lock(mtxModel);
			m_modelData.jPos = std::vector<double>(9, cnt * 0.001);
			cnt++;
		}

		// 周期时间耗尽
		if (now > wakeUpTime) {
			auto detTime = now - wakeUpTime;
			while (now > wakeUpTime)
				wakeUpTime += std::chrono::milliseconds(duration);
		}
		// 休眠
		else {
			std::this_thread::sleep_until(wakeUpTime);
		}
	}
}
