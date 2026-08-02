
#include "interpolation/InterpDispatch.h"

#include "InterpSegmentBuffer.h"

#include <iostream>
#include <fstream>
#include <cmath>
static std::ofstream jntPosOutFile("jnt_pos.txt");
static std::ofstream jntVelOutFile("jnt_vel.txt");

// 将前向声明 IMPL 定义为 InterpBuffer 的别名
struct InterpDispatcher::IMPL : public InterpBuffer {};

InterpDispatcher::InterpDispatcher() : pimpl (std::make_unique<IMPL>()) {
	// --- 默认机器人配置参数
	for (int i = 0; i < 6; ++i) {
		kineCfg.jntLowerLimit[i] = -5 - i;
		kineCfg.jntUpperLimit[i] = 5 + i;
	}
	for (int i = 0; i < 3; ++i) {
		kineCfg.extLowerLimit[i] = -8 - i;
		kineCfg.extUpperLimit[i] = 8 + i;
	}
}

// 析构函数: 编译器必须析构的代码位置看到 IMPL 的完整定义
InterpDispatcher::~InterpDispatcher() = default;

int InterpDispatcher::switch_auto(bool enable) {
	signalIn.switchMode = enable ? 1 : -1;
	return 0;
}

int InterpDispatcher::interp_enable(bool enable) {
	std::cout << "Interp " << (enable ? "enabled." : "disabled.") << std::endl;
	signalIn.interpEnable = enable;
	return 0;
}

int InterpDispatcher::run_cycle_task(InterpSignalOut& signalOut, DispatcherState& state) {
	// - 前处理
	// 第一次调用时初始化
	static PosData prePos;
	if (dispatcherStatus.CycleNum < 1) {
		// 接收初始关节角
		dispatcherStatus.dpos = state.dpos;
		// 默认进入关节模式
		switch_to_mode(0);
		prePos = state.dpos;
	}

	// 调用计数更新
	dispatcherStatus.CycleNum++;

	// - 开始信号未使能
	if (!signalIn.interpEnable) {
		state = dispatcherStatus;
		return 0;
	}

	// 跟随误差检测

	if (dispatcherStatus.autoMode > 0) {
		interp_auto_task();
	}
	else {
		interp_manual_task();
	}

	static double preVel0 = 0.0;
	for (int i = 0; i < 5; ++i) {
		jntPosOutFile << dispatcherStatus.dpos.rbtPos[i] << ", ";
		double curVel = (dispatcherStatus.dpos.rbtPos[i] - prePos.rbtPos[i]) / get_cycleTime();
		if (i == 0) {
			if (std::fabs(curVel - preVel0) > 0.1) {
				double tmp = 0.0;
			}
			preVel0 = curVel;
		}
		jntVelOutFile << (dispatcherStatus.dpos.rbtPos[i] - prePos.rbtPos[i]) / get_cycleTime() << ", ";
	}
	jntPosOutFile << dispatcherStatus.dpos.rbtPos[5] << ", ";
	jntVelOutFile << (dispatcherStatus.dpos.rbtPos[5] - prePos.rbtPos[5]) / get_cycleTime();
	for (int i = 0; i < 2; ++i) {
		jntPosOutFile << dispatcherStatus.dpos.extPos[i] << ", ";
	}
	jntPosOutFile << dispatcherStatus.dpos.extPos[2] << std::endl;
	jntVelOutFile << std::endl;
	prePos = dispatcherStatus.dpos;

	// 输出插补状态
	state = dispatcherStatus;
	return 0;
}

int InterpDispatcher::interp_auto_task() {
	// - 执行插补动作: 插补与等待状态切换需要一个周期，避免不同状态下触发的动作时序混乱
	static int interpFinish = 0;

	// 当前机器人位置
	PosData pos = dispatcherStatus.dpos;

	// 插补执行过程中: <正常插补>, <暂停过程>, <继续过程>
	if (dispatcherStatus.interpState % 2 == 1) {
		// 响应停止/暂停信号, 修改规划参数
		//if (signalIn.switchState == 0) {
		//	dispatcherStatus.interpState |= (1 << 8);
		//}

		// 执行插补
		interpFinish = pimpl->move(pos);
	}
	// 插补动作停止，等待恢复插补的信号
	else {
		// 1. <暂停> 状态，等待继续信号，加载上下文，恢复自动任务

		// 2. <等待> 状态，等待就绪信号，如事件信号、定时器信号

		// 3. 任务队列清空，进入 <完成> 状态
		// 3.1 接收新任务切换到 <插补/等待> 状态
		if (dispatcherStatus.interpState == 0 && pimpl->get_buffer_size() > 0) {
			// 运动前缓冲指令切换 <等待> 状态
			// 轨迹起点设为当前关节位置, 因为缓冲动作可能会运动导致当前点与指令起点不一致
			// 设置各轴插补曲线的起点位置、速度、加速度等
			for (int i = 0; i < 3; ++i) {
				pimpl->set_jog_constraint(i, 0, 20, 50, 500);
				pimpl->set_jog_constraint(i + 3, 0, 10, 50, 500);
				pimpl->set_jog_constraint(i + 6, 0, 100, 1000, 5000);
			}
			// 缓冲指令完成，切换 <插补> 状态
			dispatcherStatus.interpState |= 1;
		}
		// 3.2 响应非紧急状态切换信号，如手自动切换
		else if (signalIn.switchMode < 0) {
			switch_to_mode(signalIn.switchMode + 1);
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
	dispatcherStatus.dpos = pos;
	dispatcherStatus.cmdNum = pimpl->get_bufbeg();

	return 0;
}

int InterpDispatcher::interp_manual_task() {
	// 遍历轴点动使能信号
	int cmd = signalIn.switchState;
	for (int i = 0; i < 9; ++i) {
		// 修改点动状态
		int dir = cmd & 3;

		// 正向信号
		if (dir == 1) {
			pimpl->switch_jog_state(i, 1);
		}
		// 负向信号
		else if (dir == 2) {
			pimpl->switch_jog_state(i, -1);
		}
		// 异常信号重置
		else if (dir == 3) {
			pimpl->switch_jog_state(i, 0);
			signalIn.switchState &= ~(1 << (i * 2));
			signalIn.switchState &= ~(1 << (i * 2 + 1));
		}
		// 停止信号，按当前方向停止
		else {
			pimpl->switch_jog_state(i, 0);
		}

		// 执行点动插补
		double xt[4];
		int jogState = pimpl->jog_move(i, dispatcherStatus.CycleNum, get_cycleTime());
		pimpl->get_online_interp_result(i, xt);
		if (i < 6) {
			dispatcherStatus.dpos.rbtPos[i] = xt[0];
		}
		else {
			dispatcherStatus.dpos.extPos[i - 6] = xt[0];
		}

		// 更新当前插补状态
		if (jogState > 0) {
			dispatcherStatus.interpState |= (1 << (2 * i));
			dispatcherStatus.interpState &= ~(1 << (2 * i + 1));
		}
		else if (jogState < 0) {
			dispatcherStatus.interpState &= ~(1 << (2 * i));
			dispatcherStatus.interpState |= (1 << (2 * i + 1));
		}
		else {
			dispatcherStatus.interpState &= ~(1 << (2 * i));
			dispatcherStatus.interpState &= ~(1 << (2 * i + 1));
		}

		cmd = cmd >> 2;
	}

	// 不需要切换模式，可以直接返回
	if (signalIn.switchMode == 0)
		return 0;

	// 所有点动停止后，响应手自动模式切换
	if (dispatcherStatus.interpState == 0) {
		switch_to_mode(signalIn.switchMode);
	}
	// 否则不允许切换模式
	signalIn.switchMode = 0;

	return 0;
}

bool InterpDispatcher::buffer_ready() {
	return pimpl->buffer_ready();
}

double InterpDispatcher::get_cycleTime() {
	return pimpl->get_cycleTime();
}

int InterpDispatcher::add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	return pimpl->add_move_point(point, cfg, cmd);
}

// 点动模式
int InterpDispatcher::set_jog_type(int jogType) {
	signalIn.switchMode = -jogType-1;
	return 0;
}

// 下发点动信号
int InterpDispatcher::jog_move(int idx, int dir) {
	// 正向点动
	if (dir > 0) {
		int bit = idx * 2;
		signalIn.switchState |= (1 << bit);
	}
	else if (dir < 0) {
		int bit = idx * 2 + 1;
		signalIn.switchState |= (1 << bit);
	}
	else {
		int bit = idx * 2;
		signalIn.switchState &= ~(1 << bit);

		bit = idx * 2 + 1;
		signalIn.switchState &= ~(1 << bit);
	}

	return 0;
}

int InterpDispatcher::switch_to_mode(int type) {
	// 切换到自动
	if (type > 0) {
	}
	// 切换到关节
	else if (type == 0) {
		// 点位坐标系转换

		// 更新各轴插补曲线的起点位置、速度、加速度等
		for (int i = 0; i < 9; ++i) {
			double q0 = i < 6 ? dispatcherStatus.dpos.rbtPos[i] : dispatcherStatus.dpos.extPos[i - 6];
			pimpl->set_jog_constraint(i, q0, 2, 5, 5);
		}
	}

	// 切换到指定模式
	dispatcherStatus.autoMode = type;
	// 切换完成后复位当前插补状态
	dispatcherStatus.interpState = 0;

	return 0;
}
