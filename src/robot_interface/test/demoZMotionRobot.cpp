#include <windows.h>
#include<iostream>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "robot_interface/ZMotionRobot.h"
#include "RobotLogger.h"


// ¿ØÖÆ¿¨
std::shared_ptr<FSAIRobotInterface::Controller> ZController(new FSAIRobotInterface::Controller);
// »úÆ÷ÈË
std::shared_ptr<FSAIRobotInterface::RobotBase> robot(new FSAIRobotInterface::ZMotionRobot), robot2(new FSAIRobotInterface::ZMotionRobot);
FSAIRobotInterface::RobotGroupManager group;
// ¹ì¼£
DiscreteTrajectory trajList, trajList2;
TrajectoryConfig trajCfg, trajCfg2;
std::vector<int> finish;


int main() {

	ZController->lazy_connect();

	robot->set_ZController(ZController);
	robot->switch_auto(true);
	group.new_robot(robot);

	robot2->set_ZController(ZController);
	robot2->switch_auto(true);
	group.new_robot(robot2);

	group.start_thread();

	trajCfg.speed = 60;
	trajCfg.smooth = 100;
	trajList.moveJABS({ 44.975681, -39.444492, 37.797131, -0.140591, -88.208946, 134.498627, -997.599060, -216.511307, 0.000000 }, trajCfg);

	trajCfg.speed = 400;
	trajCfg.smooth = 100;
	trajList.moveLABS({ 104.777374, -393.037598, 2395.996826, -0.242211, 46.335049, 178.840378, -997.148010, -251.864197, 0.000000 }, trajCfg);

	Weave waveCfg;
	waveCfg.Id = 1;
	waveCfg.Shape = 0;
	waveCfg.Freq = 2.5;
	waveCfg.RightWidth = 2.5;
	waveCfg.LeftWidth = 2.5;
	waveCfg.Dwell_center = 0;
	waveCfg.Dwell_left = 0;
	waveCfg.Dwell_right = 0;
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Weave(waveCfg));

	trajCfg.speed = 4.5;
	trajCfg.smooth = 8;
	trajList.moveLABS({ 102.390175, -524.964783, 2396.161133, -0.105579, 46.350559, 178.960007, -1106.270996, -252.856232, 0.000000 }, trajCfg);

	trajList.moveLABS({ 100.002968, -656.891968, 2396.325684, 0.031054, 46.366066, 179.079636, -1215.394043, -253.848251, 0.000000 }, trajCfg);

	robot->push_new_trajectory(trajList);


	trajCfg2.speed = 80;
	trajCfg2.smooth = 10;
	trajList2.moveJABS({ 0,-20,20,0,90,0 }, trajCfg);
	trajList2.moveLABS({ 1111, 0, 1298, 0, -180, 0, 0, 0 }, trajCfg);

	//robot2->push_new_trajectory(trajList);

	printf("Press <Enter> to exit.\n");
	getchar();

	return 0;

}
