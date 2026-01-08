#pragma once
/* ************************************************************ *
* @brief 插补器调度层接口                                       *
*															    *
* 主要功能如下：											    *
* 1. 插补状态切换，包括手自动模式							    *
* 2. 根据插补状态执行不同的插补程序						        *
* ************************************************************* */

#include <vector>


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
	//! 模式切换: 手动，自动
	int operateMode;
	//! 状态切换: 开始/重置，暂停，继续，停止
	int machineState;
};

/**
* @brief  插补输出信号
* 
* 插补调度周期内返回的信号
*/
struct InterpSignalOut {
	//! 
};

/**
* @brief  调度器状态
*/
struct DispatcherState {
	//! 周期任务执行次数
	int CycleNum;
	//! 当前插补状态: 插补，暂停中/已暂停，继续，等待，完成/空闲
	int interpState;
	//! 警告码，触发暂停信号
	int warnCode;
	//! 异常码，触发急停信号
	int errorCode;

	//! 插补结果，规划电机角度
	std::vector<double> dpos;
};

// 插补调度
class InterpDispatcher {
	//! 调度器状态
	DispatcherState dispatcherStatus;

	/**
	* @brief  切换插补状态
	* @param  state     目标状态
	* @return 执行异常码
	*/
	int switch_interp_state(int state);

public:
	InterpDispatcher();
	~InterpDispatcher() {};

	/**
	* @brief  执行周期任务
	* @param  signalIn     [out] 输入信号
	* @param  signalOut    [out] 输出信号
	* @param  state        [out] 输出更新后的调度器状态, 第一次调用时输入初始关节角
	* @return 执行异常码
	*
	* 周期任务由逻辑层单独开线程循环调用并执行
	*/
	int run_cycle_task(InterpSignalIn& signalIn, InterpSignalOut& signalOut, DispatcherState& state);

};
