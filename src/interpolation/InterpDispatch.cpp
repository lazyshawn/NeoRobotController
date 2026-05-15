
#include "interpolation/InterpDispatch.h"
#include <iostream>

#include "InterpSegmentBuffer.h"

// 将前向声明 IMPL 定义为 InterpBuffer 的别名
struct InterpDispatcher::IMPL : public InterpBuffer {};

InterpDispatcher::InterpDispatcher() : pimpl (std::make_unique<IMPL>()) {}

// 析构函数: 编译器必须析构的代码位置看到 IMPL 的完整定义
InterpDispatcher::~InterpDispatcher() = default;

int InterpDispatcher::interp_enable(bool enable) {
	std::cout << "Interp " << (enable ? "enabled." : "disabled.") << std::endl;
	signalIn.interpEnable = enable;
	return 0;
}

int InterpDispatcher::run_cycle_task(InterpSignalOut& signalOut, DispatcherState& state) {
	// - 前处理
	// 第一次调用时初始化，接收初始关节角
	if (dispatcherStatus.CycleNum < 1) {
		dispatcherStatus.dpos = state.dpos;
	}

	// 调用计数更新
	dispatcherStatus.CycleNum++;

	// - 开始信号未使能
	if (!signalIn.interpEnable) {
		state = dispatcherStatus;
		return 0;
	}

	// 跟随误差检测

	// - 执行插补动作: 插补与等待状态切换需要一个周期，避免不同状态下触发的动作时序混乱
	static int interpFinish = 0;
	// 插补执行过程中: <正常插补>, <暂停过程>, <继续过程>
	if (dispatcherStatus.interpState % 2 == 1) {
		// 响应停止/暂停信号, 修改规划参数
		if (signalIn.switchState == 0) {
			dispatcherStatus.interpState |= (1 << 8);
		}

		// 规划新轨迹指令

		// 执行插补
		PosData pos;
		interpFinish = pimpl->move(pos);
		pimpl->get_cur_dpos(dispatcherStatus.dpos);
		//std::cout << "dpos: " << pos.rbtPos[0] << ", " << pos.rbtPos[1] << ", " << pos.rbtPos[2] << std::endl;
	}
	// 插补动作停止，等待恢复插补的信号
	else {
		// 1. <暂停> 状态，等待继续信号，加载上下文，恢复自动任务

		// 2. <等待> 状态，等待就绪信号，如事件信号、定时器信号

		// 3. 任务队列清空，进入 <完成> 状态
		// 3.1 响应非紧急状态切换信号，如手自动切换
		// 3.2 接收新任务切换到 <插补/等待> 状态
		if (dispatcherStatus.interpState == 0 && pimpl->get_buffer_size() > 0) {
			// 运动前缓冲指令切换 <等待> 状态
			// 轨迹起点设为当前关节位置
			//PosData begPos;
			//begPos.pointType = 0;
			//begPos.rbtPos = state.dpos;
			//interpBuffer.set_begin_pos(begPos);
			// 缓冲指令完成，切换 <插补> 状态
			dispatcherStatus.interpState |= 1;
		}
	}

	// - <插补> 状态完成，自动状态切换，每次完成执行仅一次
	if (interpFinish) {
		// 暂停信号，保存上下文，切换到 <暂停> 状态

		// 急停信号，保存上下文，清空剩余指令，切换到 <完成> 状态

		// 当前插补运动执行完成，依次执行缓冲动作：有缓冲读写则立即执行，有缓冲等待则进入 <等待> 状态

		// 当前指令执行完成，指令缓冲不为空，加载下一条指令
		pimpl->pop_front();
		if (pimpl->get_buffer_size() > 0) {
			std::cout << "traj finish: " << pimpl->get_bufbeg() << std::endl;
		}
		// 指令缓冲为空，进入 <完成> 状态
		else {
			std::cout << "all traj finish: " << pimpl->get_bufbeg() << std::endl;
			dispatcherStatus.interpState = 0;
		}

		interpFinish = false;
	}

	// - 输出插补结果，更新关节角
	dispatcherStatus.cmdNum = pimpl->get_bufbeg();
	state = dispatcherStatus;

	return 0;
}

bool InterpDispatcher::buffer_ready() {
	return pimpl->buffer_ready();
}

double InterpDispatcher::get_cycleTime() {
	return pimpl->cycleTime;
}

int InterpDispatcher::add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	return pimpl->add_move_point(point, cfg, cmd);
}
