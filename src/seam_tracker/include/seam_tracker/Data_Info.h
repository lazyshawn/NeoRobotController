#pragma once

#define MOTION_EXPORT

#ifdef MOTION_EXPORT   
#define MOTION_API  _declspec(dllexport)   
#else   
#define MOTION_API  extern "C" _declspec(dllimport)   
#endif  

#include <vector>

namespace motion
{
	struct CDir
	{//方向矢量
		double x;
		double y;
		double z;

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(x, y, z);
		}
	};

	struct Cart_Pnt
	{//笛卡尔点（mm），欧拉角-ZYX（弧度）
		double x;
		double y;
		double z;
		double a;
		double b;
		double c;

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(x, y, z, a, b, c);
		}
	};

	struct DH_Info
	{//长度单位（mm），角度单位（弧度）
		double a;
		double alpha;
		double d;
		double theta;

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(a, alpha, d, theta);
		}
	};

	struct Motion_Info
	{//运动参数
		double speed;//速度
		double acc;//加速度
		double dec_acc;//减加速度
		double jerk;//加加速度

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(speed, acc, dec_acc, jerk);
		}
	};

	struct Axis_Info
	{//轴参数
		int cur_encoder;//当前编码器值
		int raw_encoder;//原点处编码器值
		int pulse_ratio;//脉冲当量：1°对应的脉冲数；or 1mm对应的脉冲数
		double gear_ratio;//传动比
		double cur_val;//当前值
		double raw_val;//原点（默认值）
		double up_limit;//上限位
		double low_limit;//下限位				
		Motion_Info move_info;//运动参数

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(cur_encoder, raw_encoder, pulse_ratio, gear_ratio, cur_val, raw_val, up_limit, low_limit, move_info);
		}
	};

	struct Speed_Info
	{//移动+转动的运动参数
		Motion_Info trans_info;
		Motion_Info rotate_info;

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(trans_info, rotate_info);
		}
	};

	struct Move_Spd
	{//机器人C空间运动参数
		Speed_Info spd_manual;//手动速度
		Speed_Info spd_auto;//自动速度

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(spd_manual, spd_auto);
		}
	};

	

	struct Joint_State
	{//关节状态
		std::vector<double> robot_states;//机器人本体关节状态（弧度）
		std::vector<double> aux_states;//附加轴关节状态（弧度/mm）

		template <class Archive>
		void serialize(Archive & ar)
		{
			ar(robot_states, aux_states);
		}
	};

	

	enum class Error_Code
	{
		NOERR=0,
		ERR_DRIVER,//驱动器异常，8
		ERR_ROBOTFRAME,//机器人指令坐标错误
		ERR_AXIS_Connect,//轴通讯异常，4
		ERR_FS_LIMITOUT,//软正限位超限
		ERR_RS_LIMITOUT,//软负限位超限
		ERR_EMERGENCYSTOP,//停止状态
		COMMAND_DEVIATION_EXCESSIVE,//Endmove不等于dpos，机器人关节未跟上末端规划速度
		AXIS_PAUSE,//轴进入暂停状态

	};
	struct Robot_Status
	{
		std::vector<Error_Code>AxisState;//机器人轴状态
	};

}
