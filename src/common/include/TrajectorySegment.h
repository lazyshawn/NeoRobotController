#pragma once

#include <vector>

 // 参数类型
enum class MotionCfgType {
	SWING,     // 摆焊
};

// 摆焊参数
struct SwingInterpParam {
	static MotionCfgType type;

	// 参数Id: 区分是否属于同一组参数，用于实时参数修改
	int id = 0;

	// 设置参数
	int enable = 0;
	double freq;
	double leftWidth;
	double rightWidth;

	// 过程参数
	int state;
	double time;
	double duration;
	double pos;

	void clear();
	int get_type() const;
	//! 序列化: 写入文件或控制器时使用
	int serialize(std::vector<double>& param) const;
	//! 反序列化: 从文件或控制器读取时使用
	int deserialize(const std::vector<double>& param);
};

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
	// - 不支持实时修改的参数
	//! 运动类型: 0 关节, 1 直线, 2 圆弧
	int moveType;
	double speed = 0.0;
	double accel;
	// 结束点平滑度，起点平滑度即上一段结束点平滑度
	double smooth;
	//! 附加轴屏蔽标志
	int externalMask;
	//! 变位机屏蔽标志
	int positionerMask;

	// - 支持实时修改的参数，如焊接参数、摆焊参数等，在轨迹开始执行时更新到公共的任务参数区
	// 摆焊
	SwingInterpParam swingParam;
};

// 缓冲指令: 缓冲动作等
struct MoveCmd {
	// 自定义类型，如缓冲动作
	std::vector<int> state;
	std::vector<std::vector<double>> data;

	MoveCmd() {}

	//int push_command(int type, const std::vector<double>& msg);
};

// 轨迹段基类
class SegmentBase {
public:
	// 基础轨迹数据
	PointInfo pointInfo;
	MotionCfg motionCfg;
	MoveCmd moveCmd;

	// 设置轨迹数据
	int set_data(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd);
};
