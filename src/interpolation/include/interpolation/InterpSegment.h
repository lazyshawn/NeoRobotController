#pragma once
/* ************************************************************ *
* @brief 统一插补接口                                           *
*															    *
* 主要功能如下：											    *
* 1. 提供虚拟插补轨迹段，统一的插补接口，主要接口如下：	        *
*    - 预处理，插入轨迹时执行，处理轨迹信息并缓存               *
*    - 规划，根据预处理信息提前计算插补需要用到的数据           *
*    - 插补，计算插补周期内实际电机的目标位置                   *
* ************************************************************* */

#include <queue>
#include <memory>

#include "interpolation/InterpCurve.h"


// 轨迹类型
enum class InterpSegmentType {
	NONE,    // 未指定
	JOINT,   // 关节
	LINE,    // 直线
	CIRCLE,  // 圆弧
	BEZIER,  // 贝塞尔
};

// 点位数据
struct PosData {
	//! 点位类型: 关节，世界坐标系，本体坐标系，工件坐标系
	int pointType;
	//! 形态位
	int JntState;
	//! 本体点位
	std::vector<double> rbtPos;
	//! 附加轴点位
	std::vector<double> extPos;
	//! 变位机点位
	std::vector<double> pstPos;
};

// 轨迹点位信息
struct PointInfo {
	PosData begPos;
	PosData midPos;
	PosData endPos;
};

// 基础运动参数: 如运动类型、速度、平滑度、轴屏蔽等
struct MotionCfg {
	int moveType;
	double speed = 0.0;
	double accel;
	// 结束点平滑度，起点平滑度即上一段结束点平滑度
	double smooth;
	//! 附加轴屏蔽标志
	int externalMask;
	//! 变位机屏蔽标志
	int positionerMask;
};

// 缓冲指令: 如焊接参数、摆焊参数、缓冲动作等
struct MoveCmd {

};

/* ************************************************************ *
* @brief 轨迹处理信息                                           *
*															    *
* 1. 预处理: 接收轨迹信息时，需要根据前置或后置轨迹获取的信息   *
* 2. 规划: 补充或修改的信息                                     *
* 3. 插补: 插补完成后，保存的信息                               *
* ************************************************************* */
struct ProcessInfo {
	// 预处理完毕标志
	bool processed = false;

	// 轨迹行号
	int lineNum = -1;
	// 前平滑系数，接收当前轨迹时修改，同时修改前置轨迹的后平滑系数
	double preSmooth = -1;
	// 后平滑系数，后置轨迹插入时修改，为0时插补阶段不用考虑后续轨迹
	double postSmooth = -1;

	//! 当前段插补时间
	double maxTime = 0.0;

	// - 笛卡尔空间参数
	// 直线起点/圆弧圆心
	double knot[3];
	// 直线方向/圆弧旋转矢量
	double dir[3];
	// 直线长度/圆弧弧长
	double dist;

	// 前平滑开始处比例
	double preSmoothK = 0.0;
	// 后平滑开始处比例
	double postSmoothK = 1.0;

	// 控制点顺序与轨迹方向相同: + 轨迹点, * 平滑曲线控制点
	// + ... ---- *-*-*- ... + ... -*-*-* ---- ... ---- *-*-*- ... + ... -*-*-* ---- ... +
	//     post 0 1 2        |      0 1 2 pre      post 0 1 2      |      0 1 2 pre
	// 前段轨迹              | 当前轨迹                            | 下段轨迹
	// 后平滑3个点           | 前平滑的3个点       后平滑3个点     | 前平滑3个点

	//! 前平滑控制点
	double preCtrlPnt[6][3];
	//! 后平滑控制点
	double postCtrlPnt[6][3];
	//! 规划段长度
	double mainDist, preBlendDist, postBlendDist;
	//! 整数周期插补后的剩余距离
	double remainS = 0.0;
	//! 整数周期插补后的剩余时间
	double remainT = 0.0;
	//! 结束点速度
	double constrainedVel = 0.0;

	//! 直线段始末位置
	double segmBegDist, segmEndDist;

	// --- 结束状态
	//! 完成时间

	//! 结束点规划速度
	//! 结束点位置: 当前段位置
	double doneS = 0.0;
	double doneU = 0.0;

	void reset();
};

// 插补状态
// 1. 插补过程中频繁更新的数据
// 2, 可能需要输出的状态
struct InterpInfo {
	// --- 过程状态
	//! 插补轨迹位置: 完成(-1), 未开始(0)，前平滑，无平滑，后平滑
	int partId;
	//! 插补比列
	double schedule = 0.0;
	//! 当前过渡曲线位置参数，前平滑和后平滑共同使用该参数
	double curU = 0.0;
	//! 当前速度曲线规划的位移: 包含上一段曲线位移
	double curMoveS = 0.0;
	//! 当前周期目标位置
	PosData dpos;
};

// 插补线段基类
class InterpSegment {
protected:

public:

	// 当前插补时间，同一次前瞻的轨迹中从零开始计数，插补一次叠加一次插补周期的时间
	double curTime = 0;

	//! 各轴插补的 S 曲线
	DoubleSCurve curve;
	DoubleSCurve curvePre;

	// 轨迹数据
	PointInfo pointInfo;
	MotionCfg motionCfg;
	MoveCmd moveCmd;

	// 预处理信息
	ProcessInfo procInfo;

	// 插补信息
	InterpInfo interpInfo;

	// 设置轨迹数据
	int set_data(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd);
};



/***********************************************************************
 *                        B E Z I E R                                  *
 * ------------------------------------------------------------------- *
 * @brief 贝塞尔曲线辅助函数                                           *
 * @param  m      曲线阶数                                             *
 * @param  ctr    曲线控制点(m+1)                                      *
 ***********************************************************************/

 /******************************************
 @brief  贝塞尔曲线插值
 @param  u      待获取点位的参数值
 @param  ans    [out] 目标点位
 ******************************************/
int bezier_positioin(int m, const double ctr[][3], double u, double ans[3]);

double bezier_derivatives(int m, const double ctr[][3], double u, double ans[3]);

double bezier_dist(int m, const double ctr[][3], double a, double b, int n);

/******************************************
@brief  贝塞尔曲线插值
@param  curU   待获取点位的参数值
@param  detS   步进距离，需要足够小(detS << dist)
******************************************/
double bezier_interp(int m, const double ctr[][3], double curU, double detS);
