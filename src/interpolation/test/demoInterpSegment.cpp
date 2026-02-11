
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "interpolation/InterpSegment.h"
#include "interpolation/InterpDispatch.h"

// 指令缓存
InterpBuffer interpBuffer;
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

int test_cuvre();

int main() {
	//return test_cuvre();

	std::ofstream file, velFile;
	file.open("interp_pos.txt", std::ios::out);
	velFile.open("interp_vel.txt", std::ios::out);

	dpos.pointType = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);
	std::vector<double> dposPre = dpos.rbtPos;

	// 开启插点线程
	auto pointThreadWorker = std::thread(push_trajectory);

	// 执行插补线程
	bool interpBeg = false;
	for (int i=0; ; ++i) {
		// 传入当前角度
		dispatcherState.dpos = dpos;

		// 执行插补任务
		dispatcher.run_cycle_task(interpBuffer, signalOut, dispatcherState);

		// 更新当前角度
		dpos = dispatcherState.dpos;

		if (interpBeg) {
			// 位置输出
			file << dpos.rbtPos[0] << ", " << dpos.rbtPos[1] << ", " << dpos.rbtPos[2] << std::endl;
			// 速度输出
			for (int j = 0; j < 3; ++j) {
				dposPre[j] -= dpos.rbtPos[j];
			}
			velFile << sqrt(dposPre[0] * dposPre[0] + dposPre[1] * dposPre[1] + dposPre[2] * dposPre[2]) / interpBuffer.get_cycleTime() << std::endl;
			dposPre = dpos.rbtPos;
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
	pointInfo.endPos.rbtPos = std::vector<double>(6, 0.0);
	motionCfg.moveType = 1;
	motionCfg.speed = 2;
	motionCfg.smooth = 50;


	// 点位指令插入缓存区
	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] += 10;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] += 10;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] -= 10;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] -= 10;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	// 开始信号使能
	dispatcher.interp_enable(true);
	return 0;
}

int test_cuvre() {
	DoubleSCurve curve;

	curve.set_condition(0, 10, 7.5, 0);
	curve.plan();
	return 0;
}
