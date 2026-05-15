#pragma once

#include <deque>

#include "common/TrajectorySegment.h"
#include "common/ExportSharedAPI.h"

class SHARE_API_ SingleTrajectory : public SegmentBase {
public:
	bool isJoint();
	bool isCartesian();
};

// 上位机任务的离散轨迹指令
class SHARE_API_ DiscreteTrajectory {
	std::deque<SingleTrajectory> trajList;
	SingleTrajectory preTraj;

public:
	//! 清空轨迹
	int clear();
	//! 添加轨迹
	int add_single_traj(const SingleTrajectory& traj);
	int push_trajectory(const DiscreteTrajectory& trajectory);
	//! 弹出轨迹
	int pop();

	bool trajectory_loaded();
	SingleTrajectory get_curTraj();
	SingleTrajectory get_preTraj();
};
