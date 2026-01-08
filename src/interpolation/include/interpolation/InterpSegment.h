#pragma once

#include <queue>


// 边界条件(boundary)
struct InterpBoundary {
	double q0, q1;
	double v0, v1;
	double a0, a1;

	// 当前状态
	double qk, vk, ak, jk;

	// 减速阶段开始周期
	int kd = -1;

	// 插补阶段标志位
	int state = 0;

	InterpBoundary() {};
	InterpBoundary(double q0_, double q1_, double v0_, double v1_, double a0_, double a1_);
};


// 约束条件(constraint)
struct InterpConstraint {
	double vmax, vmin;
	double amax, amin;
	double jmax, jmin;

	//! 插补结果采样周期
	double Ts = 1e-3;
	//! 数值计算深度: 提升插补精度
	int N = 10;
	//! 数值计算周期
	double dt = Ts / N;

	//! 到位检测阈值
	double inPlacePos = 1e-4;
	double inPlaceVel = 1e-4;
	double inPlaceAcc = 1e0;

	InterpConstraint() {};
	InterpConstraint(double vmax_, double vmin_, double amax_, double amin_, double jmax_, double jmin_);
};


enum class InterpSegmentType {
	JOINT,   // 关节
	LINE,    // 直线
	CIRCLE,  // 圆弧
	BEZIER,  // 贝塞尔
};


// 插值轨迹基类
class InterpSegment {
	//! 当前状态
	double cur_q, cur_v, cur_a, cur_j;
	//! 插补阶段标识符
	int interpPhase;
	//! 当前插补轨迹参数
	double Td = 0.0, Tj2a = 0.0, Tj2b = 0.0;
	//! 当前插补周期, 开始减速周期
	int k, kb;


public:
	// 插值轨迹类型
	InterpSegmentType type;

	// 约束条件
	InterpConstraint constraint;

	// 边界条件
	std::queue<InterpBoundary> boundaryQueue;


	// 设置约束条件
	int set_kinematic_constraint(const InterpConstraint& constraint_);

	// 增加插补轨迹段
	int add_segment(const InterpBoundary& boundary);
	//int add_segment(const InterpBoundary, const InterpConstraint constraint_);

	// 弹出队首轨迹
	int pop_front_segment(InterpBoundary& boundary);

	/* **********************************************************
	* @brief  计算插补结果
	* @param  result [out]  插补点所在位置、速度、加速度、Jerk
	* @return 插补结果, 0 - 正常, other - 异常
	*********************************************************** */
	int get_interp_result(std::vector<double>& result);
};


// 轨迹类
class InterpTrajectory {
	// 轨迹段

	// 插补段
	InterpSegment segment;

public:
	//! 轨迹预处理
	// 轨迹平滑

	// 速度规划

	// 轨迹插补
};

