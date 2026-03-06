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
	double vmax = 2, amax = 5, jmax = 5;
	double vmin, amin, jmin;

	//! 曲线方向
	int sign = 1;
	// 偏移与缩放
	double scale = 1.0, offset = 0.0;
	//! 保留时间，剩余时间小于保留时间时视作插补完成
	double reserveTime = 0.0;

	//! 始末点状态
	double q0, q1, v0, v1;
	//! 不同阶段的时间
	double Tj1, Tj2, Ta, Tv, Td, T = 0;

	// - 其他常用规划参数
	//! 轨迹段最大速度、加速度
	double alima, alimd, vlim;
	// 不同阶段的位移
	double s1, s2, s3, s4, s5, s6;

	// - 保持规划插补参数
	//! 完成标识符，上次计算点位到达终点
	bool doneFlag;
	//! 规划速度
	double vp = 0.0;

	// 计算实际运动参数限制值，每次规划完后更新
	int calc_plan_param();

public:
	// 曲线初始化
	DoubleSCurve();
	/**
	* @brief  设置曲线参数
	*/
	int set_condition(double begPos, double endPos, double begVel, double endVel);
	int set_constraint(double maxVel, double maxAcc);
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
	double get_offset();


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

	/**
	* @brief  更新保存点位
	* @param  t   目标时间
	* @return 目标位置
	*/
	double move_to(double t);
	/**
	* @brief  曲线合并
	*
	* 当前段减速段与目标段加速段合并，计算合并后的最大速度与最大加速度
	*/
	double merge(const DoubleSCurve& other);
	
};
