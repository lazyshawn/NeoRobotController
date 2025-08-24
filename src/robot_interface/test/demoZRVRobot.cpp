#include <windows.h>
#include<iostream>

#include "robot_interface/ZRVRobot.h"
#include "RobotLogger.h"


// ¿ØÖÆ¿¨
std::shared_ptr<FSAIRobotInterface::Controller> ZController(new FSAIRobotInterface::Controller);
// »úÆ÷ÈË
std::shared_ptr<FSAIRobotInterface::RobotBase> robot(new FSAIRobotInterface::ZRVRobot), robot2(new FSAIRobotInterface::ZRVRobot);
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

	group.start_thread();

	trajCfg.speed = 80;
	trajCfg.smooth = 10;
	trajList.moveJABS({ -5.174500, -26.632299, 19.890400, 2.188700, -65.369301, -176.445099, -0.001900, -0.004400, 0.000000 }, trajCfg);


	trajCfg.speed = 20;
	trajCfg.smooth = 100;
	trajList.moveLABS({ 906.722900, -111.649902, 2239.583740, -3.667000, 27.206200, 176.792694, -0.001900, -0.004400, 0.000000 }, trajCfg);
	//trajCfg.speed = 200;
	//trajCfg.smooth = 100;
	//trajList.moveLABS({ 906.722900, -111.649902, 1959.583740, -3.667000, 27.206200, 176.792694, -0.001900, -0.004400, 0.000000 }, trajCfg);

	//Weave waveCfg;
	//waveCfg.Id = 1;
	//waveCfg.Freq = 1;
	//waveCfg.Shape = 0;
	//waveCfg.LeftWidth = 2;
	//waveCfg.RightWidth = 2;
	//waveCfg.Dwell_left = 100;
	//waveCfg.Dwell_right = 100;
	//waveCfg.Dwell_center = 0;
	//waveCfg.Dwell_type = 1;
	//trajCfg.add_appendix(FSAIRobotInterface::serialize_Weave(waveCfg));

	//trajCfg.speed = 10;
	//trajCfg.smooth = 80;
	//trajList.moveLABS({ 1111, 0, 1298, 0, -180, 0, 0, 0 }, trajCfg);

	robot->push_new_trajectory(trajList);

	printf("Press <Enter> to exit.\n");
	getchar();

	return 0;

}
