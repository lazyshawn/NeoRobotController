#pragma once
/* ************************************************************ *
* @brief 插补曲线功能实现                                       *
*															    *
* 主要功能如下：											    *
* 1. 提供不同类型的基础插补曲线，如双S, 梯形, 多项式等	        *
* ************************************************************* */

#include <cmath>

//双S曲线插补, 七段规划
class DoubleSCurve {
	//! 运动学约束
	double m_vmax = 2, m_amax = 5, m_jmax = 5;
	double m_vmin, m_amin, m_jmin;

	//! 曲线方向
	int m_sign = 1;
	// 偏移与缩放：新曲线时间戳与旧曲线时间戳对应关系为: tn = K to + dt
	double m_scale = 1.0, m_offset = 0.0;
	//! 保留时间，剩余时间小于保留时间时视作插补完成
	double m_reserveTime = 0.0;

	//! 始末点状态
	double m_q0, m_q1, m_v0, m_v1;
	//! 不同阶段的时间
	double m_Tj1, m_Tj2, m_Ta, m_Tv, m_Td, m_T = 0;

	// - 其他常用规划参数
	//! 轨迹段最大速度、加速度
	double m_alima, m_alimd, m_vlim;
	// 不同阶段的位移
	double m_s1, m_s2, m_s3, m_s4, m_s5, m_s6;

	// - 保持规划插补参数
	//! 完成标识符，上次计算点位到达终点
	bool m_doneFlag;
	//! 规划速度，后续移除，存入m_xt中
	double m_vp = 0.0;

	// --- 在线插补参数
	//! 在线插补状态: +/- 运动方向，1 - 加速/匀速阶段, 2 - 减速阶段
	int m_onlineState;
	long m_decCnt = 0;
	//! 当前插补结果 [xt, vt, at, jt]
	double m_xt[4];

	// 计算实际运动参数限制值，每次规划完后更新
	int calc_plan_param();

public:
	DoubleSCurve();

	void clear();
	/**
	* @brief  设置曲线参数
	*/
	int set_condition(double begPos, double endPos, double begVel, double endVel);
	int set_constraint(double maxVel, double maxAcc, double maxJerk = -1);
	/**
	* @brief  设置保留时间
	* @param  time    保留时间(s)
	*/
	int set_reserve_time(double time);
	/**
	* @brief  曲线规划
	* @return 规划结果
	*    - 0  正常返回
	*    - 1  规划超速
	*
	* 计算各阶段的时间
	*/
	int plan();
	/**
	* @brief  通过设定各个阶段的时间进行规划
	*
	* 前提条件：
	* 1. 首末点速度为 0，v0 = v1 = 0
	* 2. vmax, amax 均能达到，设定值无效
	*/
	int plan_by_duration(double Tall, double Tacc, double Tjerk);

	// 获取最大规划时间
	double get_duration();
	double get_Ta();
	double get_Td();
	double get_Tv();
	double get_vp();
	/**
	* @brief  计算曲线
	*
	* 计算给定时间下的曲线位置
	*/
	double get_pos(double t);
	bool done();
	double get_offset() const;
	double get_scale() const;

	int get_onlineState();
	int set_onlineState(int state);
	/**
	* @brief  在线插补
	* @param  phase    插补阶段: 0 - 加速，1 - 减速
	*/
	int online_interp(double dt);
	// 获取当前插补状态
	int get_cur_state(double state[4]);
	// 规划在线插补减速阶段: 计算各个减速阶段的时间，返回终点位置
	double plan_decccel_online_interp(double Tdi[3]);
	// 使用减速规划参数
	int apply_deccel_plan(double Tdi[3]);

	/**
	* @brief  曲线缩放与偏移
	* @param  dt  时间偏移，正值曲线左移，负值曲线右移
	* @param  k   曲线缩放，起点位置不变
	*
	* tn = k*t + dt，先缩放再偏移
	*/
	int displacement(double dt, double k);
	/**
	* @brief  计算整数插补周期后的剩余距离
	* @param  dt  单个插补周期的时间
	*/
	double get_remain_dist(double dt);
	double get_remain_time(double dt);

	/**
	* @brief  提速最大速度值
	* @param  ds  提速位移
	* 
	* 不考虑减速阶段，按 vmax, amax, jmax 约束，计算在给定距离 ds 和初始速度 v0 时可以达到的最终速度 vlim
	*/
	double get_max_speed(double ds);
	/**
	* @brief  计算给定距离到运动结束需要的时间
	* @param  ds  剩余位移
	*/
	double calc_time_PiTPe(double ds);
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

/******************************************
@brief  贝塞尔曲线导数
@param  u      待获取点位的参数值
@param  ans    [out] xyz三方向的导数值
@return 导数模长
******************************************/
double bezier_derivatives(int m, const double ctr[][3], double u, double ans[3]);

/******************************************
@brief  贝塞尔曲线长度
@param  a, b   需要求长度的区间位置
@param  n      积分段数
@return 曲线长度
******************************************/
double bezier_dist(int m, const double ctr[][3], double a, double b, int n);

/******************************************
@brief  贝塞尔曲线插值
@param  curU   待获取点位的参数值
@param  detS   步进距离，需要足够小(detS << dist)
******************************************/
double bezier_interp(int m, const double ctr[][3], double curU, double detS, int num);
