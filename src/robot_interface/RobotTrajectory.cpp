#include "robot_interface/RobotTrajectory.h"

bool SingleTrajectory::isJoint() {
	return motionCfg.moveType == 0;
}
bool SingleTrajectory::isCartesian() {
	return motionCfg.moveType > 0;
}

int DiscreteTrajectory::clear() {
	trajList.clear();
	return 0;
}

int DiscreteTrajectory::add_single_traj(const SingleTrajectory& traj) {
	trajList.push_back(traj);
	return 0;
}

int DiscreteTrajectory::push_trajectory(const DiscreteTrajectory& trajectory) {
	for (auto& traj : trajectory.trajList) {
		trajList.push_back(traj);
	}
	return 0;
}

int DiscreteTrajectory::pop() {
	trajList.pop_front();
	return 0;
}

bool DiscreteTrajectory::trajectory_loaded() {
	return trajList.empty();
}
SingleTrajectory DiscreteTrajectory::get_curTraj() {
	return trajList.front();
}
SingleTrajectory DiscreteTrajectory::get_preTraj() {
	return preTraj;
}
