
#include "robot_interface/RobotTrajectory.h"
#include <iostream>

int main() {
	std::cout << "hello world" << std::endl;

	DiscreteTrajectory traj;
	TrajectoryConfig trajCfg;
	trajCfg.set_speed(100);
	trajCfg.set_smooth(10);

	TrajectoryPoint p0(9);
	p0.mainPoint = { 509.164001, 6284.015625, 2386.361084, -0.562525, 46.997433, 97.512436, 6293.027832, 75.713524, 0.000000 };
	p0.trajType = TrajType::Line;

	TrajectoryPoint p1(9);
	p1.mainPoint = { 10,10,10 };
	p1.trajType = TrajType::Line;

	TrajectoryPoint p2(9);
	p2.mainPoint = { 690.952148, 6306.531250, 2386.343750, -0.561845, 46.997440, 97.394173, 6293.027832, 254.533478, 0.000000 };
	p2.auxPoint = { 600.056641, 6295.273438, 2386.343750, -0.562185, 46.997437, 97.453308, 6293.027832, 165.123505, 0.000000 };
	p2.trajType = TrajType::Arc;

	traj.moveCABS(p2.auxPoint, p2.mainPoint, trajCfg);
	traj.set_preTraj(p0);
	traj.calc_traj_info();

	//auto ans = partition_trajectory(p0, p2, 1.0/2, 1, 0);
	////traj.moveLABS(p0.mainPoint, trajCfg);
	//traj.clear();
	//traj.moveCABS(ans.auxPoint, ans.mainPoint, trajCfg);
	//traj.set_preTraj(p0);
	//traj.calc_traj_info();

	printf("Press <Enter> to exit.\n");
	getchar();
	return 0;
}
