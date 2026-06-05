
#include <windows.h>
#include <iostream>
#include <Eigen/Dense>
//#include <mutex>
//#include <condition_variable>
//#include <unordered_set>
//#include <atomic>

#include "robot_interface/CoopRobotManager.h"
//#include "robot_interface/BufferSynchronizer.h"

#include "CoopRobotBase.h"
#include "NeoRobot.h"
#include "RobotLogger.h"


namespace FSAIRobotInterface {

static const double DT_PI = 3.14159265358979323846;

/* *************************** RobotGroupManager *************************** */
RobotGroupManager::RobotGroupManager() :
	pimpl(std::make_unique<IMPL>())
{
}

RobotGroupManager::~RobotGroupManager() {
	// 退出所有线程
	this->stop();
}

// --- 隐藏实现
struct RobotGroupManager::IMPL {

	//! 协同就绪状态: 0 未就绪, 1 已就绪
	std::vector<int> syncReadyState;
	//! 协同就绪状态: <<type, num>, ...>
	//std::vector<Sync_Config> syncState;
	//! 机器人等待状态: <bit> <<robot, num>, ...>
	//std::vector<std::unordered_set<int>> waitState;

	//! 机器人队列
	std::vector<std::shared_ptr<RobotBase>> robotList;

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
	//! 机器人分组
	std::vector<std::vector<int>> disableGroup;
	//! 共用轴
	std::pair<int, int> sharedAxisState;

	IMPL();

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
	/**
	* @brief  机器人组暂停
	*/
	int robot_group_pause(int idx);
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

RobotGroupManager::IMPL::IMPL() {
	cmdThreadDone = true;
	bufferThreadDone.store(true);
}

int RobotGroupManager::new_robot(int type)
{
	std::shared_ptr<RobotBase> robot;
	if (type == NEOROBOT) {
		robot = std::make_shared<NeoRobot>();
	}

	pimpl->robotList.push_back(robot);
	// 设置别名ID

	pimpl->syncReadyState.push_back(0);
	//pimpl->waitState.push_back({});
	pimpl->coopState.push_back(0);
	pimpl->disableGroup.push_back({});

	return 0;
}

// --- 需要代理的接口
int RobotGroupManager::get_rt_robot_status(int idx, RobotStatus& robotStatus) {
	return pimpl->robotList[idx]->get_rt_robot_status(robotStatus);
}

int RobotGroupManager::push_new_trajectory(int idx, const DiscreteTrajectory& trajectory) {
	pimpl->robotList[idx]->push_new_trajectory(trajectory);
	return 0;
}

int RobotGroupManager::switch_auto(int idx, bool enable) {
	pimpl->robotList[idx]->switch_auto(enable);
	return 0;
}

int  RobotGroupManager::set_jogType(int robotIdx, int type) {
	pimpl->robotList[robotIdx]->set_jog_type(type);
	return 0;
}

int RobotGroupManager::jog_moving(int robotIdx, int type, int idx, int dir, int move) {
	pimpl->robotList[robotIdx]->jog_moving(type, idx, dir, move);
	return 0;
}


int RobotGroupManager::start_thread() {
	// 绑定成员函数和 this 指针
	pimpl->updateThreadWorker = std::thread(&RobotGroupManager::IMPL::updateStatusThread, pimpl.get());
	return 0;
}


int RobotGroupManager::stop() {

	pimpl->workerHealthy = false;
	// 结束工作线程
	if (pimpl->updateThreadWorker.joinable())
		pimpl->updateThreadWorker.join();
	if (pimpl->cmdThreadWorker.joinable())
		pimpl->cmdThreadWorker.join();

	// 状态刷新线程使能
	for (auto& robot : pimpl->robotList) {
		// 唤醒等待中的线程
		robot->notify_waiting_robot();
	}

	return 0;
}


void RobotGroupManager::IMPL::processCommandThread() {
	cmdThreadDone.store(false);
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;


	// 指令执行线程
	while (workerHealthy) {
		
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 开始执行轨迹: 设定上条轨迹，计算协同轨迹时间
			robotList[i]->set_ready_for_consistent_traj(coopState[i]);

			// 更新每台机器人当前的同步设置
			set_group_sync_config(i);
		}

		// 轨迹动作处理
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 更新同步状态
			update_sync_state(i);
		}

		// Group 状态处理
		// 协同轨迹速度更新
		//correct_sync_speed();
		// 所有机器人空闲(协同完成)，取消地轨屏蔽
		if (robot_group_idle()) {
			// 打印取消屏蔽日志
			//for (size_t i = 0; i < robotList.size(); ++i) {
			//	robotList[i]->set_axisIdxMask({});
			//}
			if (sharedAxisState.second > 0) {
				LOG4CPLUS_INFO(RobotLog::getLogger(), "Unset mask of shared axis: " << sharedAxisState.first);
			}
			sharedAxisState.second = -1;
		}
		
		// 指令下发
		for (size_t i = 0; i < robotList.size(); ++i) {

			// 指令缓存不为空
			while (!robotList[i]->trajectory.empty()) {

				// 机器人状态警告,不清空轨迹: 处于暂停状态
				if (robot_warning(i))
					break;

				// 机器人出现错误
				if (robot_error(i))
					break;

				// 运动完成
				if (robot_idle(i))
					break;

				// 获取当前轨迹
				auto curTraj = robotList[i]->trajectory.get_curTraj();
				// 获取上一条轨迹
				auto preTraj = robotList[i]->trajectory.get_preTraj();

				// 轨迹一致性不满足
				if (!robotList[i]->consistent_traj_ready(coopState[i]))
					break;

				// --- 轨迹下发前的重新处理
				if (curTraj.isCartesian()) {
					// 重新计算轨迹参数
					//robotList[i]->trajectory.calc_traj_info();

					// 插入轨迹
					robotList[i]->insert_task_traj();

					// 拆分轨迹
					robotList[i]->separate_trajectory();

					// 重新计算轨迹参数
					//robotList[i]->trajectory.calc_traj_info();
					// 更新轨迹
					curTraj = robotList[i]->trajectory.get_curTraj();
				}

				// --- 轨迹下发检测
				// 指令缓存检测
				if (!robotList[i]->remain_buffer_free()) {
					if (get_bit(coopState[i], 2) == 0) {
						set_bit(coopState[i], 2, true);
						LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " buffer not enough: "
							<< "Move to: ");
					}
					break;
				}
				else {
					set_bit(coopState[i], 2, false);
				}

				// 起弧等待
				if (get_bit(coopState[i], 11)) {
					// 起弧成功取消置位
					if (robotList[i]->check_arc_on()) {
						set_bit(coopState[i], 11, 0);
						// 重设起点
						//preTraj.set_trajType(TrajType::None);
						//robotList[i]->trajectory.set_preTraj(preTraj);
						LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " arc on, continue to move.");
					}
					// 未起弧则跳过下发
					break;
				}

				// 需要等待同步和协同: 空闲 + 同步号相同
				if (!robot_sync_ready(i)) {
					if (get_bit(coopState[i], 3) == 0) {
						LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " not synced: ");
						set_bit(coopState[i], 3, true);
					}
					break;
				}
				else {
					set_bit(coopState[i], 3, false);
				}

				// IO 同步标志复位
				reset_wait_state(i);

				//// 检查地轨指令是否已经下发
				//if (robotList[i]->find_command_axis(curTraj, sharedAxisState.first) >= 0 && sharedAxisState.second >= 0 && sharedAxisState.second != i) {
				//	// 屏蔽当前机器人的公用地轨轴
				//	robotList[i]->set_axisIdxMask({ sharedAxisState.first });
				//}
				//else {
					// 地轨轴指令跟随当前机器人发送
					sharedAxisState.second = i;
				//}

				// --- 开始下发轨迹
				// 缓冲动作补充参数: 部分参数需要在轨迹下发前才能确定，特别是需要在下位机补充执行轨迹的动作
				auto action = curTraj.motionCfg.moveAction;
				robotList[i]->rewrie_actioin_param(action);

				// 执行运动前动作
				robotList[i]->execute_move_action(action.before, 0);

				// 下发运动指令
				if (curTraj.isJoint()) {
					ret = robotList[i]->execute_single_joint();
				}
				else if (curTraj.isCartesian()){
					ret = robotList[i]->execute_single_cartesian();
				}

				// 下发异常处理
				if (ret != 0) {
					LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " send command failed: " << ret);
					robotList[i]->set_upperStatus(0, 0x20);
					robot_group_stop(i);
					break;
				}

				// 执行运动后动作
				robotList[i]->execute_move_action(action.after, 1);


				// 轨迹下发后的处理
				robotList[i]->process_after_send_traj();

				// 起弧等待置位
				//if (curTraj.waitArcOn > 0) {
				//	set_bit(coopState[i], 11, true);
				//	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " waiting arc on.");
				//}

				// 记录当前轨迹编号，用于轨迹完成后的触发动作
				//curTraj.lineNum = robotList[i]->get_lineNum();
				robotList[i]->trajHistory.add_single_traj(curTraj);

			}

		}

		// 所有指令下发完毕
		bool taskFinish = true;
		for (size_t i = 0; i < robotList.size(); ++i) {
			if (!robotList[i]->trajectory.empty()) {
				taskFinish = false;
				break;
			}
		}
		if (taskFinish) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "Process Command Thread Terminated.");
			cmdThreadDone.store(true);
			return;
		}

		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		auto now = std::chrono::steady_clock::now();

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


void RobotGroupManager::IMPL::updateStatusThread() {
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;
	// 保存机器人状态
	statusList.resize(robotList.size());
	LOG4CPLUS_INFO(RobotLog::getLogger(), "Process Command Thread Begin.");

	// 指令执行线程
	while (workerHealthy) {

		// 开始时间
		auto tmpStart = std::chrono::steady_clock::now();
		//LOG4CPLUS_INFO(RobotLog::getLogger(), "Start at:" << std::chrono::duration_cast<std::chrono::milliseconds>(tmpStart - start).count());

		std::vector<int> dt(4, 0);
		// 更新状态
		auto t0 = std::chrono::steady_clock::now();
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 更新机器人状态 (唯一更新途径)
			robotList[i]->update_rt_robot_status();
			// 获取机器人状态: 保证当前周期使用相同的机器人状态
			robotList[i]->get_rt_robot_status(statusList[i]);
		}
		auto t1 = std::chrono::steady_clock::now();
		dt[0] = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();


		// 获取下位机缓冲数据
		t0 = std::chrono::steady_clock::now();
		//for (size_t i = 0; i < robotList.size(); ++i) {
		//	robotList[i]->get_slave_buffer();
		//}
		t1 = std::chrono::steady_clock::now();
		dt[1] = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();



		// 获取下位机缓冲数据
		t0 = std::chrono::steady_clock::now();
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 轨迹到位处理
			robot_in_place_command(i);

			// 捕获下位机日志
			robotList[i]->capture_controller_log();
		}
		t1 = std::chrono::steady_clock::now();
		dt[2] = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();



		// 指令缓存检测，指令下发线程未运行则开启
		t0 = std::chrono::steady_clock::now();
		for (size_t i = 0; i < robotList.size(); ++i) {

			// 可以下发任务
			bool startCmdThread = true;

			// 机器人状态警告,不清空轨迹: 处于暂停状态
			if (robot_warning(i)) {
				// 关联机器人同步进入暂停
				pause_coop_robot(i);
				startCmdThread = false;
				//continue;
			}

			// 机器人出现错误
			if (robot_error(i)) {
				robotList[i]->notify_waiting_robot();
				// 关联机器人同步进入暂停
				pause_coop_robot(i);
				startCmdThread = false;
				//continue;
			}

			// 运动完成，清空轨迹
			if (robot_idle(i)) {
				robotList[i]->notify_waiting_robot();
				startCmdThread = false;
				//continue;
			}

			// 指定轨迹完成唤醒
			//if (robotList[i]->get_notifyType() > 0) {
			//	robotList[i]->notify_waiting_robot();
			//}

			// 机器人未异常，指令下发程序，轨迹未下发完成
			if (startCmdThread && cmdThreadDone && !robotList[i]->trajectory.empty()) {
				if (cmdThreadWorker.joinable())
					cmdThreadWorker.join();
				cmdThreadWorker = std::thread(&RobotGroupManager::IMPL::processCommandThread, this);
				break;
			}
		}
		t1 = std::chrono::steady_clock::now();
		dt[3] = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();


		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		auto now = std::chrono::steady_clock::now();
		auto tmpEnd = std::chrono::steady_clock::now();

		// 周期时间耗尽
		if (now > wakeUpTime) {
			while (now > wakeUpTime)
				wakeUpTime += std::chrono::milliseconds(duration);
		}


		// 休眠
		//std::this_thread::sleep_until(wakeUpTime);
		std::this_thread::sleep_for(wakeUpTime - now);
	}

}


void RobotGroupManager::IMPL::readSlaveBufferThread() {
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 20;


	// 指令执行线程
	while (workerHealthy) {

		// 开始时间
		auto tmpStart = std::chrono::steady_clock::now();
		std::vector<int> dt(4, 0);

		// 获取下位机缓冲数据
		auto t0 = std::chrono::steady_clock::now();
		for (size_t i = 0; i < robotList.size(); ++i) {
			robotList[i]->get_slave_buffer();
		}
		auto t1 = std::chrono::steady_clock::now();
		dt[1] = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();


		if (bufferThreadDone) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "Read Slave Buffer Thread erminated.");
			return;
		}


		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		auto now = std::chrono::steady_clock::now();

		// 周期时间耗尽
		if (now > wakeUpTime) {
			auto detTime = now - wakeUpTime;
			//LOG4CPLUS_INFO(RobotLog::getLogger(), "Read Slave Buffer Thread Cycle time exausted:" << std::chrono::duration_cast<std::chrono::milliseconds>(detTime).count());
			while (now > wakeUpTime)
				wakeUpTime += std::chrono::milliseconds(duration);
		}
		// 休眠
		else {
			std::this_thread::sleep_until(wakeUpTime);
		}
	}
}


int RobotGroupManager::slave_buffer_stream(bool enable) {
	if (enable) {
		// 线程已经开启
		if (!pimpl->bufferThreadDone) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "Read Slave Buffer Thread Already Running.");
			return 0;
		}

		pimpl->bufferThreadDone.store(false);
		if (pimpl->bufferThreadWorker.joinable())
			pimpl->bufferThreadWorker.join();
		// 绑定成员函数和 this 指针
		pimpl->bufferThreadWorker = std::thread(&RobotGroupManager::IMPL::readSlaveBufferThread, pimpl.get());

		LOG4CPLUS_INFO(RobotLog::getLogger(), "Read Slave Buffer Thread Begin.");
	}
	else {
		pimpl->bufferThreadDone.store(true);
		if (pimpl->bufferThreadWorker.joinable())
			pimpl->bufferThreadWorker.join();

		LOG4CPLUS_INFO(RobotLog::getLogger(), "Read Slave Buffer Thread End.");
	}
	return 0;
}


bool RobotGroupManager::IMPL::robot_error(int idx) {

	// 下位机无异常，上位机无异常
	if ((statusList[idx].lowerStatus >> 2) == 0 && statusList[idx].upperStatus == 0) {
		set_bit(coopState[idx], 1, false);
		return false;
	}
	else if (get_bit(coopState[idx], 1) == 0) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " error occur: "
			<< statusList[idx].lowerStatus << ", " << statusList[idx].upperStatus << ", " << coopState[idx] << ".\n"
			<< "Pos: " << vector_to_string(statusList[idx].cPos));

		set_bit(coopState[idx], 1, true);
	}

	return true;

}

bool RobotGroupManager::IMPL::robot_warning(int idx) {

	if (get_bit(statusList[idx].lowerStatus, 1) == 0) {
		set_bit(coopState[idx], 0, false);
	}
	// 机器人处于暂停状态
	else {
		if (get_bit(coopState[idx], 0) == 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " warning occur: "
				<< statusList[idx].lowerStatus << ", " << statusList[idx].upperStatus << ", " << coopState[idx] << ".\n"
				<< "Pos: " << vector_to_string(statusList[idx].cPos));

			vector_to_string(statusList[idx].cPos);

			set_bit(coopState[idx], 0, true);
		}
		return true;
	}

	// 上位机未触发暂停
	//if (get_bit(coopState[idx], 8)) {
	//	return true;
	//}

	return false;

}

bool RobotGroupManager::IMPL::robot_idle(int idx) {

	// 无运动，轨迹完成，无运动缓冲
	if (statusList[idx].lowerStatus == 0 && robotList[idx]->task_assigned_completed() && robotList[idx]->trajectory.empty()) {

		if (get_bit(coopState[idx], 7) == 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " task complete.");

			// 重新设定上条轨迹类型
			//TrajectoryPoint point;
			//auto preTraj = robotList[idx]->trajectory.get_preTraj();
			//point = preTraj.get_point();
			//point.trajType = TrajType::None;
			//preTraj.set_point(point);

			set_bit(coopState[idx], 7, true);
		}

		return true;
	}

	set_bit(coopState[idx], 7, false);

	return false;
}


bool RobotGroupManager::IMPL::robot_group_idle(const std::vector<int>& ids) {

	std::vector<int> idSet = ids;

	// 输入为空时检查所有机器人
	if (idSet.size() == 0) {
		idSet = std::vector<int>(robotList.size(), 0);
		for (size_t i = 0; i < idSet.size(); ++i) {
			idSet[i] += i;
		}
	}

	for (size_t i = 0; i < idSet.size(); ++i) {

		// 输入异常
		if (idSet[i] > robotList.size() || idSet[i] < 0) {
			return false;
		}

		int idx = idSet[i];
		// 协同机器人运动未完成
		if (!robotList[i]->task_assigned_completed() || statusList[idx].lowerStatus != 0) {
			return false;
		}

	}

	return true;
}

void RobotGroupManager::IMPL::set_group_sync_config(int robotIdx) {

}


void RobotGroupManager::IMPL::update_sync_state(int robotIdx) {

}


bool RobotGroupManager::IMPL::robot_sync_ready(int robotIdx) {

	return true;
}


void RobotGroupManager::IMPL::correct_sync_speed() {

	return;
}


void RobotGroupManager::IMPL::robot_in_place_command(int robotIdx) {

	// 无已下发轨迹
	if (robotList[robotIdx]->trajHistory.empty())
		return;

}


int RobotGroupManager::IMPL::pause_coop_robot(int idx) {

	// 暂停关联机器人
	for (auto& robot : disableGroup[idx]) {
		// 关联机器人未暂停
		if ((statusList[robot].upperStatus >> 2) > 0) {
			robotList[robot]->task_pause();
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robot << " paused because of error in robot " << idx);
			// 标记为被动暂停
			robotList[robot]->set_upperStatus(0, 0x02);
		}
	}

	return 0;
}


int RobotGroupManager::IMPL::calc_sync_duration(int robotIdx) {

	return 0;
}



int RobotGroupManager::IMPL::robot_group_resume(int idx) {
	// 当前机器人继续
	robotList[idx]->task_resume();
	// 暂停触发标志复位
	set_bit(coopState[idx], 8, false);

	// 同步继续绑定机器人
	for (auto& robot : disableGroup[idx]) {
		robotList[robot]->task_resume();
	}
	return 0;
}

int RobotGroupManager::IMPL::robot_group_pause(int idx) {
	// 当前机器人暂停
	robotList[idx]->task_pause();
	// 暂停触发标志置位
	set_bit(coopState[idx], 8, true);

	// 同步暂停绑定机器人
	for (auto& robot : disableGroup[idx]) {
		robotList[robot]->task_pause();
	}

	return 0;
}

int RobotGroupManager::IMPL::robot_group_update_saved_pos(const std::vector<int>& idxList) {
	bool paused = false;

	while (!paused) {
		paused = true;
		for (auto& idx : idxList) {
			// 运动中且未暂停
			if (get_bit(statusList[idx].lowerStatus, 0) == 1 && get_bit(statusList[idx].lowerStatus, 1) == 0) {
				paused = false;
				break;
			}
		}
		if (!paused) {
			// 延时
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	// 暂停成功，保存状态
	for (auto& idx : idxList) {
		robotList[idx]->trigger_action(6, {});
	}
	return 0;
}

int RobotGroupManager::IMPL::robot_group_clear_task(int idx) {
	// 暂停触发标志复位
	//set_bit(coopState[idx], 8, false);
	coopState[idx] = 0;

	robotList[idx]->task_clear();

	return 0;
}

int RobotGroupManager::IMPL::robot_group_stop(int idx) {
	// 暂停触发标志复位
	set_bit(coopState[idx], 8, false);

	robotList[idx]->emergency_stop();

	return 0;
}


void RobotGroupManager::IMPL::reset_wait_state(int robotIdx) {

}


std::string vector_to_string(const std::vector<double>& data, int fixed) {
	std::string str;
	if (data.size() < 1) {
		return str;
	}

	for (int i = 0; i < data.size(); ++i) {
		auto dataStr = std::to_string(data[i]);

		if (fixed > 0) {
			str += dataStr.substr(0, dataStr.find(".") + fixed + 1);
		}
		else if (fixed == 0) {
			str += dataStr.substr(0, dataStr.find("."));
		}
		else {
			str += dataStr;
		}

		if (i < data.size() - 1) {
			str += ", ";
		}
	}

	return str;
}

std::string vector_to_string(const std::vector<int>& data, int fixed) {
	std::vector<double> ans(data.size(), 0);

	for (size_t i = 0; i < data.size(); ++i) {
		ans[i] = static_cast<int>(data[i]);
	}

	return vector_to_string(ans, 0);
}

int set_bit(int& state, int idx, bool enable) {

	// 判断总共有多少位

	// 位掩码
	int mask = 0xFFFFFF - (1 << idx);
	state = (state & mask) + (enable << idx);

	return 0;
}

int get_bit(int state, int idx) {
	return (state >> idx) % 2;
}

} // namespace ZMotionRobot
