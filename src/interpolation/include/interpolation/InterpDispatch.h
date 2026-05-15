#pragma once
/* ************************************************************ *
* @brief 插补器调度层接口                                       *
*															    *
* 主要功能如下：											    *
* 1. 每个插补周期执行任务，根据插补状态执行不同的插补程序       *
* 2. 插补状态切换，包括手自动模式							    *
* 3. 管理与用户层的插补信号交互  							    *
* ************************************************************* */

#include <vector>
#include <memory>

#include "common/TrajectorySegment.h"
#include "common/ExportSharedAPI.h"

/**
* @brief  插补输入信号
*
* 逻辑层对调度层的管理信号，触发模式和状态切换等
* - 模式切换:
* 1. 手动模式: 执行点动任务
* 2. 自动模式: 自动执行任务，不响应手动运动信号
* - 状态切换:
* 1. 开始/重置: 恢复所有状态
* 2. 暂停; 任务停止，保存停止时刻的上下文
* 3. 继续: 恢复暂停时的上下文，运动恢复
* 4. 停止: 任务停止，清空任务，其他状态保留
*/
struct InterpSignalIn {
	//! 开始信号
	bool interpEnable = false;
	//! 模式切换: 手动，自动
	int switchMode;
	//! 状态切换: 重置，暂停，继续，停止
	int switchState;
};

/**
* @brief  插补输出信号
* 
* 插补调度周期内返回的信号
*/
struct InterpSignalOut {
	//! 插补器状态
	// 0      | 4    | 5    | 8        | 9      | 10
	// 插补中 | 暂停 | 等待 | 正常插补 | 暂停中 | 断点继续
	int interpState;
	//! 警告码，触发暂停信号
	int warnCode;
	//! 异常码，触发急停信号
	int errorCode;
	//! 当前插补指令号
	int cmdNum;
};

/**
* @brief  调度器状态
*/
struct DispatcherState {
	//! 周期任务执行次数
	int CycleNum;
	//! 当前插补行号
	int cmdNum;
	//! 当前插补状态: 插补，暂停中/已暂停，继续，等待，完成/空闲
	int interpState;
	//! 警告码，触发暂停信号
	int warnCode;
	//! 异常码，触发急停信号
	int errorCode;

	//! 插补结果，规划关节位置
	PosData dpos;
};

// 插补调度
class SHARE_API_ InterpDispatcher {
	//! 调度器状态
	DispatcherState dispatcherStatus;
	//! 逻辑层输入信号，封装后由上层调用进行切换
	InterpSignalIn signalIn;

	// Impl 模式前置声明，将轨迹操作单独封装
	struct Impl;
	std::unique_ptr<Impl> pimpl;

public:
	InterpDispatcher();
	~InterpDispatcher();
	
	// --- 外部信号
	// 手自动切换信号
	// 开始信号使能
	int interp_enable(bool enable);
	// 暂停信号使能

	/**
	* @brief  执行周期任务
	* @param  cmdQueue     预处理后的指令缓冲
	* @param  signalOut    [out] 输出信号
	* @param  state        [out] 输出更新后的调度器状态, 第一次调用时输入初始关节角
	* @return 异常码
	*
	* 周期任务由逻辑层单独开线程循环调用并执行
	*/
	int run_cycle_task(InterpSignalOut& signalOut, DispatcherState& state);

	// --- 轨迹操作代理
	bool buffer_ready();
	double get_cycleTime();
	int add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd);
};
