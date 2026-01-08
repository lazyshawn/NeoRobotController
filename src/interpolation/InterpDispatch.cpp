
#include "interpolation/InterpDispatch.h"
#include <iostream>

InterpDispatcher::InterpDispatcher() {
	dispatcherStatus.CycleNum = 0;
	dispatcherStatus.interpState = 0;
}

int InterpDispatcher::switch_interp_state(int state) {
	return 0;
}

int InterpDispatcher::run_cycle_task(InterpSignalIn& signalIn, InterpSignalOut& signalOut, DispatcherState& state) {
	int interpStatus = dispatcherStatus.interpState;

	// - 前处理
	// 第一次调用时初始化，接收初始关节角
	if (dispatcherStatus.CycleNum < 1) {
		dispatcherStatus.dpos = state.dpos;
	}

	// 调用计数更新
	dispatcherStatus.CycleNum++;

	// 跟随误差检测

	// - 执行插补动作: 插补与等待状态切换需要一个周期，避免不同状态下触发的动作时序混乱
	// 插补执行过程中: <正常插补>, <暂停过程>, <继续过程>
	if (interpStatus % 2 == 1) {
		// 响应停止/暂停信号, 修改规划参数

		// 执行插补
	}
	// 未执行插补，满足条件后下一个周期恢复插补
	else {
		// 1. <暂停> 状态，等待继续信号，加载上下文，恢复自动任务

		// 2. <等待> 状态，等待就绪信号，如事件信号、定时器信号

		// 3. 任务队列清空，进入 <完成> 状态
		// 3.1 响应非紧急手动状态切换信号
		// 3.2 接收新任务切换到 <插补/等待> 状态
	}

	// - <插补> 状态完成，自动状态切换
	if (1) {
		// <暂停> 状态完成，保存上下文，切换到 <暂停> 状态

		// 插补运动执行完成，依次执行缓冲动作：有缓冲等待则进入 <等待> 状态，有缓冲读写则立即执行

		// 当前指令执行完成，保持 <插补> 状态，进行下一段轨迹插补

		// 轨迹指令执行完成，进入 <完成> 状态
	}

	// - 插补完成，更新关节角
	std::cout << dispatcherStatus.CycleNum << std::endl;

	return 0;
}
