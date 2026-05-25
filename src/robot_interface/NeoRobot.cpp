
#include "interpolation/InterpDispatch.h"

#include "NeoRobot.h"

namespace FSAIRobotInterface {

// 调度器
static InterpDispatcher dispatcher;
// 调度器状态
static DispatcherState dispatcherState;
// 调度器输出信号
static InterpSignalOut signalOut;

NeoRobot::NeoRobot() {
	// 初始位置
	dispatcherState.dpos.pointType = 0;
	dispatcherState.dpos.rbtPos = std::vector<double>(6, 0);
	dispatcherState.dpos.extPos = std::vector<double>(3, 0);
	dispatcherState.dpos.pstPos = {};

	// 开启插补线程
	interpThreadWorker = std::thread(&NeoRobot::interp_thread, this);

	// 开始信号使能
	dispatcher.interp_enable(true);
}

NeoRobot::~NeoRobot() {
	interpWorkerHealthy = false;
	if (interpThreadWorker.joinable()) {
		interpThreadWorker.join();
	}
}

void NeoRobot::interp_thread() {
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = dispatcher.get_cycleTime() * 1e3;

	// --- 循环变量
	// 当前关节角
	PosData dpos = dispatcherState.dpos;
	// 上次插补关节角
	PosData dposPre = dpos;

	// 指令执行线程
	while (interpWorkerHealthy) {
		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		auto now = std::chrono::steady_clock::now();
		auto tmpEnd = std::chrono::steady_clock::now();

		// 加锁并执行插补任务
		{
			std::lock_guard<std::mutex> lock(mtxMotion);
			dispatcher.run_cycle_task(signalOut, dispatcherState);
		}

		// 周期时间耗尽
		if (now > wakeUpTime) {
			while (now > wakeUpTime)
				wakeUpTime += std::chrono::milliseconds(duration);
		}

		// 休眠
		std::this_thread::sleep_for(wakeUpTime - now);
	}
}

//! 执行缓冲运动
int NeoRobot::execute_move_action(const std::vector<std::pair<int, std::vector<double>>>& actionList, int flag) {
	return 0;
}

//! 轨迹下发后处理
int NeoRobot::process_after_send_traj() {
	return 0;
}

int NeoRobot::execute_single_joint() {
	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();

	dispatcher.add_move_point(curTraj.pointInfo, curTraj.motionCfg, curTraj.moveCmd);

	// 弹出轨迹
	trajectory.pop();

	return 0;

}
int NeoRobot::execute_single_cartesian() {
	auto curTraj = trajectory.get_curTraj();
	auto preTraj = trajectory.get_preTraj();

	dispatcher.add_move_point(curTraj.pointInfo, curTraj.motionCfg, curTraj.moveCmd);

	// 弹出轨迹
	trajectory.pop();

	return 0;
}

//! 剩余缓冲检测
int NeoRobot::remain_buffer_free() {
	return dispatcher.buffer_ready();
}

//! 读取断点信息
int NeoRobot::read_saved_status(RobotStatus& status) {
	return 0;

}

int NeoRobot::set_ready_for_consistent_traj(int& state) {

	return 0;
}

int NeoRobot::consistent_traj_ready(int& state) {
	return 1;
}

int NeoRobot::separate_trajectory() {

	return 0;
}

int NeoRobot::update_rt_robot_status() {
	// 该线程与仿真线程分别读写状态，会有冲突，需要加锁保护
	std::lock_guard<std::mutex> lock(mtxMotion);

	robotStatus.lowerStatus = 0;
	robotStatus.jPos = dispatcherState.dpos.all_to_vector();
	robotStatus.autoMode = dispatcherState.autoMode;

	return 0;
}


// --- 对外接口
//! 下发运动补偿
int NeoRobot::move_compensate(const std::vector<float>& det) {

	return 0;
}

//! 下发自动任务
int NeoRobot::push_new_trajectory(DiscreteTrajectory trajList) {
	trajectory.push_trajectory(trajList);
	return 0;
}

int NeoRobot::switch_auto(bool enableAuto) {
	dispatcher.switch_auto(enableAuto);
	return 0;
}

int NeoRobot::switch_enable(bool enable) {
	return 0;
}

int NeoRobot::set_manual_speed(float ratio) {
	return 0;
}

//! 设置点动类型
int NeoRobot::set_jog_type(int type) {
	printf("jogType = %d\n", type);
	dispatcher.set_jog_type(type);
	return 0;
}
//! 点动执行
int NeoRobot::jog_moving(int type, int idx, int dir, int move) {
	printf("NeoRobot - Jog: %d, %d, %d, %d\n", type, idx, dir, move);

	dispatcher.jog_move(idx, move > 0 ? dir : 0);
	return 0;
}

//! 任务暂停
int NeoRobot::task_pause() {
	return 0;
}
//! 任务继续
int NeoRobot::task_resume() {
	return 0;
}
//! 任务清空
int NeoRobot::task_clear() {
	return 0;
}
//! 急停
int NeoRobot::emergency_stop() {
	return 0;
}

}
