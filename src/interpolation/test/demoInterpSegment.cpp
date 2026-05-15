
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "interpolation/InterpDispatch.h"

// 调度器
InterpDispatcher dispatcher;
// 调度器状态
DispatcherState dispatcherState;
// 调度器输出信号
InterpSignalOut signalOut;
// 点位数据
PointInfo pointInfo;
MotionCfg motionCfg;
MoveCmd moveCmd;
// 当前关节角
PosData dpos;

// 插点线程
int push_trajectory();


int main() {
	//return test_cuvre();
	//return test_dynamic();

	std::ofstream file, velFile;
	file.open("interp_pos.txt", std::ios::out);
	velFile.open("interp_vel.txt", std::ios::out);
	std::ofstream extFile, extVelFile;
	extFile.open("interp_ext.txt", std::ios::out);
	extVelFile.open("interp_ext_vel.txt", std::ios::out);

	dpos.pointType = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);
	dpos.extPos = std::vector<double>(6, 0.0);
	PosData dposPre = dpos;

	// 开启插点线程
	auto pointThreadWorker = std::thread(push_trajectory);

	// 执行插补线程
	bool interpBeg = false;
	for (int i=0; ; ++i) {
		// 传入当前角度
		dispatcherState.dpos = dpos;

		// 执行插补任务
		dispatcher.run_cycle_task(signalOut, dispatcherState);

		// 更新当前角度
		dpos = dispatcherState.dpos;

		if (interpBeg) {
			// 位置输出
			file << dpos.rbtPos[0] << ", " << dpos.rbtPos[1] << ", " << dpos.rbtPos[2] << std::endl;
			// 速度输出
			for (int j = 0; j < 3; ++j) {
				dposPre.rbtPos[j] -= dpos.rbtPos[j];
			}
			velFile << sqrt(dposPre.rbtPos[0] * dposPre.rbtPos[0] + dposPre.rbtPos[1] * dposPre.rbtPos[1] + dposPre.rbtPos[2] * dposPre.rbtPos[2]) / dispatcher.get_cycleTime() << std::endl;

			// 附加轴位置
			extFile << dpos.extPos[0] << std::endl;
			// 附加轴速度
			for (int j = 0; j < 3; ++j) {
				dposPre.extPos[j] -= dpos.extPos[j];
			}
			extVelFile << sqrt(dposPre.extPos[0] * dposPre.extPos[0] + dposPre.extPos[1] * dposPre.extPos[1] + dposPre.extPos[2] * dposPre.extPos[2]) / dispatcher.get_cycleTime() << std::endl;

			dposPre = dpos;
		}

		if (dispatcherState.interpState % 2 == 1) {
			interpBeg = true;
		}
		else {
			interpBeg = false;
		}
	}

	// 线程回收
	if (pointThreadWorker.joinable()) {
		pointThreadWorker.join();
	}

	file.close();
	return 0;
}


int push_trajectory() {
	pointInfo.midPos.rbtPos = std::vector<double>(6, 0.0);
	pointInfo.endPos.rbtPos = std::vector<double>(6, 0.0);
	pointInfo.endPos.extPos = std::vector<double>(3, 0.0);
	motionCfg.moveType = 1;
	motionCfg.speed = 2;
	motionCfg.smooth = 40;

	SwingConfig swingCfg;
	swingCfg.enable = 1;
	swingCfg.freq = 1.0;
	swingCfg.leftWidth = swingCfg.rightWidth = 0.5;
	motionCfg.swingParam = swingCfg;

	// 点位指令插入缓存区
	while (!dispatcher.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] += 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	dispatcher.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!dispatcher.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	motionCfg.moveType = 2;
	pointInfo.midPos.rbtPos[0] = pointInfo.endPos.rbtPos[0] + 8;
	pointInfo.midPos.rbtPos[1] = pointInfo.endPos.rbtPos[1] + 8;
	pointInfo.endPos.rbtPos[1] += 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	dispatcher.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!dispatcher.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	motionCfg.moveType = 1;
	pointInfo.endPos.rbtPos[0] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	dispatcher.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!dispatcher.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	dispatcher.add_move_point(pointInfo, motionCfg, moveCmd);

	// 开始信号使能
	dispatcher.interp_enable(true);

	return 0;
}
