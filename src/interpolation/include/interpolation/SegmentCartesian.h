#pragma once
/* ************************************************************ *
* @brief 笛卡尔空间轨迹段插补                                   *
*															    *
* 主要功能如下：											    *
* 1. 笛卡尔空间速度规划，如前瞻回溯、奇异点规避等      	        *
* ************************************************************* */

#include "interpolation/InterpSegment.h"

// 笛卡尔空间插补线段
class CartesianInterpSegment : public InterpSegment {

public:
	//! 主轴插补的 S 曲线
	DoubleSCurve curve;
	DoubleSCurve curvePre;
	
	//! 曲线长度

	// 预处理
	virtual int prehandle(InterpSegment& pre) override;
	// 规划
	int plan(InterpSegment& pre, InterpSegment& next) override;
	// 插补
	int move(PosData& pos) override;
	// 停止规划
	int stop_plan() override;
	// 重置
	int reset() override;
	// 获取当前时间
	double get_current_time() override;
};
