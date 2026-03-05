#include <windows.h>
#include<iostream>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "robot_interface/FSAIRobot.h"
#include "RobotLogger.h"


// 控制卡
std::shared_ptr<FSAIRobotInterface::Controller> ZController(new FSAIRobotInterface::Controller);
// 机器人
std::shared_ptr<FSAIRobotInterface::RobotBase> robot(new FSAIRobotInterface::FSAIRobot), robot2(new FSAIRobotInterface::FSAIRobot);
FSAIRobotInterface::RobotGroupManager group;
// 轨迹
DiscreteTrajectory trajList, trajList2;
TrajectoryConfig trajCfg, trajCfg2;
FSAIRobotInterface::Sync_Config synCfg, synCfg2;
std::vector<int> finish;

namespace shawn_test {
	void robot_trajectory();

	void set_task();

	void run_task();

	void sync_test();

	void task_test();

	void send_command_1();

	void send_command_2();

	void monitor_robot_status();
}


int main() {
	ZController->lazy_connect();

	//char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	//strcpy(cmdbuff, "BASE(0,1,2,3,4,5)\nMOVERV_JABS(0,-20,20,0,90,0)");
	//ZController->sendCmd(cmdbuff, cmdbuffAck, 1);
	//return 0;

	//// 下载 ZAR
	//ZController->load_basic_project("D:\\CIMC\\FSAI_Teaching_Free_Weld_System\\zmotion_basic\\main.zar", 0);
	// 导出配置文件
	//ZController->export_config("config.txt");
	// 加载配置文件
	//ZController->load_config("config.txt");

	robot->set_ZController(ZController);
	//robot2->set_ZController(ZController);

	//std::vector<float> pos;
	//robot->get_multilayer_pos(pos);

	group.new_robot(robot);
	//group.new_robot(robot2);
	//group.set_shared_axis(6, { 0,1,2,3 });
	//group.disableGroup = { { 0, 1 } };
	
	// 开启任务线程
	group.start_thread();
	shawn_test::run_task();

	printf("Press <Enter> to exit.\n");
	getchar();
	group.stop();

	return 0;
}


void shawn_test::robot_trajectory() {
	DiscreteTrajectory trajList;
	TrajectoryConfig config;
	config.set_speed(10);
	config.set_smooth(5);

	trajList.moveJABS({ 0, 0, 0, 90, 0, 90 }, config);
	trajList.moveJABS({ 0, -20, 20, 0, 90, 0 }, config);
}

void shawn_test::set_task() {
	return;
}

void shawn_test::run_task() {
	// 开启监控线程
	auto monitor = std::thread(&monitor_robot_status);
	monitor.detach();

	// 切换自动模式
	robot->switch_auto(true);
	//robot2->switch_auto(true);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// 压入轨迹
	//sync_test();
	task_test();

	// 开启监控线程
	auto cmdThread1 = std::thread(&send_command_1);
	cmdThread1.detach();
	// 开启监控线程
	//auto cmdThread2 = std::thread(&send_command_2);
	//cmdThread2.detach();

	monitor_robot_status();
}


void shawn_test::send_command_1() {
	finish.push_back(0);

	int i = 0, trajIdx = 0;
	while (true) {

		int state = robot->wait_auto_task_stop();
		if (state == 0) {
			std::cout << "机器人空闲 1" << std::endl;
			if (trajIdx == 0) {
				// 轨迹指令压栈
				robot->push_new_trajectory(trajList);
				std::cout << "发送轨迹 1" << std::endl;
				trajIdx++;
			}
			else {
				break;
			}
		}
		else {
			std::cout << "异常唤醒 1" << i++ << std::endl;
		}
	}

	finish[0] = 1;
}

void shawn_test::send_command_2() {
	finish.push_back(0);

	int i = 0, trajIdx = 0;
	while (true) {

		int state = robot2->wait_auto_task_stop();
		if (state == 0) {
			std::cout << "机器人空闲 2" << std::endl;
			if (trajIdx == 0) {
				// 轨迹指令压栈
				robot2->push_new_trajectory(trajList2);
				std::cout << "发送轨迹 2" << std::endl;
				trajIdx++;
			}
			else {
				break;
			}
		}
		else {
			std::cout << "异常唤醒 2" << i++ << std::endl;
		}
	}

	finish[1] = 1;
}

void shawn_test::monitor_robot_status() {

	std::vector<FSAIRobotInterface::RobotStatus> preStatus(group.robotList.size());
	bool cycleFlag = true;

	while (true) {
		int sum = 0;
		for (size_t i = 0; i < finish.size(); ++i) {
			sum += finish[i];
		}
		cycleFlag = sum == finish.size() ? false : true;

		for (size_t i = 0; i < group.robotList.size(); ++i) {
			auto robot = group.robotList[i];

			//if (!robot->get_enableRefresh()) {
			//	std::cout << "线程结束: " << robot->get_aliasId() << std::endl;
			//	//return;
			//}

			FSAIRobotInterface::RobotStatus curStatus;
			robot->get_rt_robot_status(curStatus);

			// 跳过重复报错
			if (preStatus[i].lowerStatus != curStatus.lowerStatus || preStatus[i].upperStatus != curStatus.upperStatus) {
				if ((curStatus.lowerStatus & 0x02) == 2) {
					std::cout << "机器人暂停" << i << std::endl;
				}
				else if ((curStatus.lowerStatus & 0x04) == 4) {
					std::cout << "机器人停止" << i << std::endl;
				}

				if ((curStatus.lowerStatus & 0x10) != 0) {
					std::cout << "轴状态异常" << i << std::endl;
				}

				if ((curStatus.upperStatus & 0x01) != 0) {
					std::cout << "线程结束" << i << std::endl;
					return;
				}
				if ((curStatus.upperStatus & 0x10) != 0) {
					std::cout << "手动自动模式不匹配" << i << std::endl;
					return;
				}
			}

			preStatus[i] = curStatus;
		}
	}

}


void shawn_test::sync_test() {

	// 机器人1
	trajCfg.set_speed(50);
	trajCfg.set_smooth(5);

	trajList.moveJABS({ 0.000000,0.000000,20.000000,-20.000000,90.000000,0.000000,0.000000 }, trajCfg);
	
	synCfg.clear();
	synCfg.add_sync_item(2, 1, 50);
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg));
	trajList.moveLABS({ 1265.883057, -39.332199, 996.359070, 20, -160, 0 }, trajCfg);

	synCfg.clear();
	synCfg.add_sync_item(3, 1, 5);
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg));
	trajList.moveLABS({ 1365.883057, -39.332199, 996.359070, 20, -160, 0 }, trajCfg);
	synCfg.clear();
	synCfg.add_sync_item(3, 1, 0);
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg));
	trajList.moveLABS({ 1465.883057, -39.332199, 996.359070, 20, -160, 0 }, trajCfg);
	synCfg.clear();
	synCfg.add_sync_item(3, 1, -1);
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg));
	trajList.moveLABS({ 1565.883057, -39.332199, 996.359070, 20, -160, 0 }, trajCfg);


	// 机器人2
	trajCfg2.set_speed(80);
	trajCfg2.set_smooth(5);

	trajList2.moveJABS({ 0.000000,0.000000,-20.000000,-20.000000,90.000000,0.000000,0.000000 }, trajCfg2);

	synCfg2.clear();
	synCfg2.add_sync_item(2, 0, 50);
	trajCfg2.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg2));
	trajList2.moveLABS({ 3142.9975, -2052.8186, 1082.9772, 20, -166, 168 }, trajCfg2);

	synCfg2.clear();
	synCfg2.add_sync_item(3, 0, 5);
	trajCfg2.add_appendix(FSAIRobotInterface::serialize_Sync_Config(synCfg2));
	trajList2.moveLABS({ 3242.9975, -2052.8186, 1082.9772, 20, -166, 168 }, trajCfg2);

}

void shawn_test::task_test() {
	
	// 机器人1
	trajCfg.set_speed(80);
	trajCfg.set_smooth(0);

	//trajList.moveJABS({ 10,-20,20,0,90,0,0 }, trajCfg);
	trajCfg.saveSeq = 10;
	trajCfg.rewriteId = 1;
	trajList.moveLABS({ 879.5140, 155.3090, 659.0740, -169.9990, - 44.9990, 179.9990, 0,0,0 }, trajCfg);
	//trajList.moveCABS({ 979.5140, 55.3090, 659.0740, -169.9990, -44.9990, 179.9990 }, { 1179.5140, 155.3090, 659.0740, -169.9990, -44.9990, 179.9990 }, trajCfg);
	trajCfg.rewriteId = -1;

	Arc_WeldingParaItem weldCfg;
	weldCfg.Id = 1;
	weldCfg.WeldingCrt_Spd = 123;
	//trajCfg.add_appendix(FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg));

	Weave waveCfg;
	waveCfg.Id = 1;
	waveCfg.Freq = 1;
	waveCfg.Shape = 4;
	waveCfg.LeftWidth = 2;
	waveCfg.RightWidth = 2;
	waveCfg.Dwell_left = 0;
	waveCfg.Dwell_right = 0;
	waveCfg.Dwell_center = 0;
	waveCfg.Dwell_type = 1;
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Weave(waveCfg));

	Track trackCfg;
	trackCfg.Id = 1;
	trackCfg.Lr_enable = 1;
	trackCfg.Ud_enable = 1;
	//trajCfg.add_appendix(FSAIRobotInterface::serialize_Track(trackCfg));

	ArcPitBackfill pitFillCfg;
	pitFillCfg.enable = true;
	pitFillCfg.distance = 10;
	trajCfg.add_appendix(FSAIRobotInterface::serialize_ArcPitBackfill(pitFillCfg));

	FSAIRobotInterface::Move_Action action;
	// 等待
	//action.actionAfter.push_back({ 1, { -1, 0, 100 } });
	// 起弧
	//action.actionBefore.push_back({ 2, FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg).second });
	// 息弧
	action.actionAfter.push_back({ 3, {} });
	// 寻位
	//action.actionAfter.push_back({ 5, {869.5140, 255.3090, 659.0740, 10, 879.5140, 255.3090, 659.0740, 10} });
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Move_Action(action));

	trajCfg.saveSeq = 20;
	trajCfg.set_speed(10);
	trajList.moveLABS({ 879.5140, 255.3090, 659.0740, -169.9990, -44.9990, 179.9990, 0,0,0 }, trajCfg);


	trajCfg.set_speed(20);
	waveCfg.Id = 0;
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Weave(waveCfg));
	action.actionBefore.clear();
	action.actionAfter.clear();
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Move_Action(action));
	//trajList.moveJABS({ 10,-20,20,0,90,0,0 }, trajCfg);
	trajCfg.saveSeq = 30;
	trajList.moveLABS({ 889.5140, 255.3090, 659.0740, -169.9990, -44.9990, 179.9990, 0,0,0 }, trajCfg);

	//trajList.moveLABS({ 911, 0, 1298, 180, 0, 180, 0 }, trajCfg);
	//trajList.moveCABS({ 911, 100, 1298, 180, 0, 180, 0 }, { 1011, 0, 1298, 180, 0, 180, 0 }, trajCfg);

	//trajList.moveCABS({ 316, -464, 2390.829590, -0.988111, 46.992050, 127.864029 },
	//	{ 387, -494, 2390.242188, -0.711897, 46.995243, 108.287041 }, trajCfg);


	//trajList2.moveLABS({ -1095, 40, 1700, -160, 20, 0, 0 }, trajCfg);

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	group.robotList[0]->modify_point_in_buffer(1, { 0,1,2,3,4 });
}
