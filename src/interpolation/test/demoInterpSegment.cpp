
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

	std::ofstream file;
	file.open("interpolation.txt", std::ios::out);

	dpos.pointType = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);

	// 开启插点线程
	auto pointThreadWorker = std::thread(push_trajectory);

	// 执行插补线程
	for (int i=0; ; ++i) {
		// 传入当前角度
		dispatcherState.dpos = dpos;

		// 执行插补任务
		dispatcher.run_cycle_task(interpBuffer, signalOut, dispatcherState);

		// 更新当前角度
		dpos = dispatcherState.dpos;

		if (dispatcherState.interpState % 2 == 1) {
			file << dpos.rbtPos[0] << std::endl;
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
	motionCfg.speed = 5;
	motionCfg.smooth = 100;

	// 开始信号使能
	dispatcher.interp_enable(true);

	// 点位指令插入缓存区
	for (int i = 0; i < 15; ++i) {
		while (!interpBuffer.buffer_ready()) {
			// 等待
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		pointInfo.begPos = pointInfo.endPos;
		pointInfo.endPos.rbtPos[0] += 10/* * pow(-1, i)*/;
		interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);
	}

	return 0;
}

int test_cuvre() {
	DoubleSCurve curve;

	curve.set_condition(0, 10, 7.5, 0);
	curve.plan();
	return 0;
}
