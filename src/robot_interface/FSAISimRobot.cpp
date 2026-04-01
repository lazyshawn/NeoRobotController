
#include "robot_interface/FSAISimRobot.h"

#include "RobotLogger.h"
#include <iostream>

namespace FSAIRobotInterface {

	//! 获取下发指令轴号，主要用于确定运动主轴
	std::vector<int> FSAISimRobot::get_execute_axis() {
		return {0, 1, 2, 3, 4, 5, 6, 7};
	}

	// 机器人状态
	int FSAISimRobot::update_rt_robot_status() {
		return 0;
	}
    
	int FSAISimRobot::get_all_robot_status(RobotStatus& status) {
		return 0;
	}

	int FSAISimRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveJABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		return 0;
	}

	int FSAISimRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relMove, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveLABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& end, const std::vector<int>& mask) {
		return 0;
	}

	int FSAISimRobot::moveC(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		return 0;
	}
	int FSAISimRobot::moveCABS(const std::vector<int>& axis, const std::vector<float>& beg, const std::vector<float>& mid, const std::vector<float>& end, int imode, const std::vector<int>& mask) {
		return 0;
	}

	int FSAISimRobot::move_compensate(const std::vector<float>& det) {
		return 0;
	}

	/**
	* @brief  设置手动速度比率
	* @param  ratio    速度比率(0-100)
	* @return 设置状态: 0 - 设置成功; 1 - 未处于手动模式
	*/
	int FSAISimRobot::set_manual_speed(float ratio) {
		return 0;
	}

	// 自动任务
	int FSAISimRobot::update_swing_config() {
		return 0;
	}
	int FSAISimRobot::update_track_config() {
		return 0;
	}
	int FSAISimRobot::update_welder_config() {
		return 0;
	}
	int FSAISimRobot::get_remain_buffer() {
		return 0;
	}

	int FSAISimRobot::push_new_trajectory(DiscreteTrajectory trajList) {
		return 0;
	}

	int FSAISimRobot::execute_single_joint() {
		return 0;
	}
	
	int FSAISimRobot::execute_single_cartesian() {
		return 0;
	}

	// 设置运动行号
	int FSAISimRobot::send_line_num(int axis, const SingleTrajectory &curTraj) {
		return 0;
	}

	// 剩余缓冲检测
	int FSAISimRobot::remain_buffer_free() {
		return 0;
	}

	// 一致性轨迹预处理，可以连续下发的轨迹
	int FSAISimRobot::set_ready_for_consistent_traj(int& state) {
		return 0;
	}

	// 一致性轨迹就绪
	int FSAISimRobot::consistent_traj_ready(int& state) {
		return 0;
	}

	int FSAISimRobot::separate_trajectory() {
		return 0;
	}

	/* *************************** 上层自定义接口 *************************** */
	/**
	* @brief  读取保存点位
	*/
	int FSAISimRobot::read_saved_status(RobotStatus& status) {
		return 0;
	}

	int FSAISimRobot::get_local_world_dpos(std::vector<float>& dpos) {
		return 0;
	}
	int FSAISimRobot::cpos_base_to_world(std::vector<float>& cPos) {
		return 0;
	}

	int FSAISimRobot::switch_auto(bool enableAuto) {
		return 0;
	}
	int FSAISimRobot::switch_enable(bool enable) {
		return 0;
	}

	int FSAISimRobot::reset_line_num() {
		return 0;
	}
	int FSAISimRobot::set_jog_type(int type) {
		return 0;
	}
	int FSAISimRobot::jog_moving(int type, int idx, int dir, int move) {
		return 0;
	}
	int FSAISimRobot::save_task_status(bool enable, int inBuffer) {
		return 0;
	}

	int FSAISimRobot::task_pause() {
		return 0;
	}
	int FSAISimRobot::task_resume() {
		return 0;
	}
	int FSAISimRobot::task_stop() {
		return 0;
	}
	int FSAISimRobot::emergency_stop() {
		return 0;
	}

	// 设备操作
	int FSAISimRobot::device_operation() {
		return 0;
	}

	int FSAISimRobot::execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag) {
		return 0;
	}
	int FSAISimRobot::process_after_send_traj() {
		return 0;
	}

	/* *************************** 获取运动轴号 *************************** */
	// 设置上条轨迹类型
	int FSAISimRobot::set_previous_trajectory(const SingleTrajectory& preTraj) {
		return 0;
	}

	/* *************************** 初始化 *************************** */
	FSAISimRobot::FSAISimRobot() {
        simThreadDone.store(true); 
        // 开启仿真线程
        std::thread sim_thread(&FSAISimRobot::sim_thread, this);
        sim_thread.detach();
	}
	FSAISimRobot::~FSAISimRobot() {
	}

	// 写入缓冲寄存器
	int FSAISimRobot::write_buffer_register(int flag) {
		return 0;
	}

    void FSAISimRobot::sim_thread() {
        simThreadDone.store(false);
        // 获取当前时间戳
        auto start = std::chrono::steady_clock::now();
        // 下次唤醒时间
        auto wakeUpTime = start;
        // 线程周期(ms)
        long long duration = 1000;

        while (!simThreadDone) {
            
            // 设置下次唤醒时间
            wakeUpTime += std::chrono::milliseconds(duration);
            auto now = std::chrono::steady_clock::now();

            std::cout << "now: " << std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() << std::endl;

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

} // namespace robot_interface
