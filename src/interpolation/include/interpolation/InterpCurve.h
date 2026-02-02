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
	double vmax = 10, amax = 10, jmax = 30;
	double vmin, amin, jmin;

	//! 曲线方向
	int sign = 1;
	// 偏移与缩放
	double scale = 1.0, offset = 0.0;

	//! 始末点
	double q0, q1;
	double v0, v1;
	//! 不同阶段的时间
	double Tj1, Tj2, Ta, Tv, Td, T;

	//! 轨迹段最大速度、加速度
	double alima, alimd, vlim;

	// - 保持规划插补参数
	//! 完成标识符，保存点位到达终点
	bool done;
	//! 当前保存点位
	double qt;

public:
	// 曲线初始化
	DoubleSCurve();
	/**
	* @brief  设置曲线参数
	*/
	int set_condition(double begPos, double endPos, double begVel, double endVel);
	/**
	* @brief  曲线规划
	* @return 规划结果
	*    - 0  正常返回
	*    - 1  规划超速
	*
	* 计算各阶段的时间
	*/
	int plan();
	// 获取最大规划时间
	double get_duration();
	double get_Ta();
	double get_Td();
	/**
	* @brief  计算曲线
	*
	* 计算给定时间下的曲线位置
	*/
	double get_pos(double t);
	/**
	* @brief  曲线缩放与偏移
	* @param  dt  时间偏移，正值曲线左移，负值曲线右移
	* @param  k   曲线缩放，起点位置不变
	*
	* tn = k*t + dt，先缩放再偏移
	*/
	int displacement(double dt, double k);
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
