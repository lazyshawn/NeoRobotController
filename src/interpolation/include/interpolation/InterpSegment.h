#pragma once

#include <queue>
#include <memory>


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
};


enum class InterpSegmentType {
	JOINT,   // 关节
	LINE,    // 直线
	CIRCLE,  // 圆弧
	BEZIER,  // 贝塞尔
};


// 速度曲线
class SCurve {
	//! 基本曲线参数

public:
	// 曲线初始化
	int curve_init();
	// 曲线同步
	// 曲线规划
	// 曲线求解
	// 修改起始时间
};

// 插补器缓存数据
class InterpBuffer {
	//! 点位指令信息: 运动类型由上层指令重新指定
	InterpSegmentType segmType;
	//! 预处理信息

	//! 插补结果
public:
	InterpSegmentType get_segment_type();
};

// 插补线段基类
class InterpSegment {

public:
	//! 插补器缓存数据, 所有类共用
	static std::shared_ptr<InterpBuffer> interpBuf;


	// - 虚函数
	// 预处理: 插入点位时执行
	virtual int prehandle() = 0;
	// 规划: 插补开始前执行，同时输出第一个插补点
	virtual int plan() = 0;
	// 插补: 插补点位
	virtual int move() = 0;
	// 停止规划: 修改插补规划
	virtual int stop_plan() = 0;
	// 重置
	virtual int reset() = 0;
	// 获取当前时间
	virtual int get_current_time() = 0;
};


// 关节空间插补线段
class JointInterpSegment : public InterpSegment {
	//! 各轴插补的 S 曲线

public:
	// 预处理
	virtual int prehandle() override;
	// 规划
	int plan() override;
	// 插补
	int move() override;
	// 停止规划
	int stop_plan() override;
	// 重置
	int reset() override;
	// 获取当前时间
	int get_current_time() override;
};

//// 笛卡尔空间插补线段
//class CartesianInterpSegment : public InterpSegment {
//	//! 点位坐标系
//
//public:
//	// 规划
//	int plan() override;
//	// 插补
//	int move() override;
//	// 停止规划
//	int stop_plan() override;
//	// 重置
//	int reset() override;
//	// 获取当前时间
//	int get_current_time() override;
//};
