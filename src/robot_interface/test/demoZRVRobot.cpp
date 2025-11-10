#include <windows.h>
#include<iostream>

#include "robot_interface/ZRVRobot.h"
#include "RobotLogger.h"


// 控制卡
std::shared_ptr<FSAIRobotInterface::Controller> ZController(new FSAIRobotInterface::Controller);
// 机器人
std::shared_ptr<FSAIRobotInterface::RobotBase> robot(new FSAIRobotInterface::ZRVRobot), robot2(new FSAIRobotInterface::ZRVRobot);
FSAIRobotInterface::RobotGroupManager group;
// 轨迹
DiscreteTrajectory trajList, trajList2;
TrajectoryConfig trajCfg, trajCfg2;
std::vector<int> finish;


/*
// test1.cpp : 定义控制台应用程序的入口点。
//
#include "robot_interface/ZMotionController.h"
//#include "stdafx.h"
#include <windows.h>
#include "zmotion.h"
#include "zauxdll2.h"

void commandCheckHandler(const char *command, int ret)
{
	if (ret)//非0则失败
	{
		printf("%s return code is %d\n", command, ret);
	}

}

int main()
{

	ZMC_HANDLE handle = NULL;                   //连接句柄

	char MotionID[32] = "127.0.0.1";				//改模式当前对字符串无要求，可填空字符串
	//int ret = ZAux_FastOpen(2, MotionID, 1000, &handle);	//连接仿真器
	int ret=ZAux_FastOpen(5, "MotionRT1",1000 ,&handle);	//连接MotionRT

	if (ERR_SUCCESS != ret)
	{
		printf("控制器连接失败！\n");
		handle = NULL;
		getchar();
		return -1;
	}
	printf("控制器连接成功！\n");
	ZAux_SetTraceFile(4, "sdf");

	char ackbuff[2048] = {0};
	ret = ZAux_DirectCommand(handle, "Table(7003)=0.000", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("Table", ret);
	ret = ZAux_DirectCommand(handle, "Table(7007)=-1", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("Table", ret);

	Sleep(1000);
	ret = ZAux_DirectCommand(handle, "Table(7051)=-1", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("Table", ret);
	Sleep(1000);

	ret = ZAux_DirectCommand(handle, "FORCE_SPEED(0)=0.8", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("FORCE_SPEED  ", ret);

	ret = ZAux_DirectCommand(handle, "ZSMOOTH(0)=10", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("ZSMOOTH  ", ret);

	ret = ZAux_DirectCommand(handle, "BASE(0,1,2,3,4,5,6,7,8,)\n MOVERV_JABS(-5.174500,-26.632299,19.890400,2.188700,-65.369301,-176.445099,-0.001900,-0.004400,0.000000,)", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("MOVERV_JABS   ", ret);


	

	//ret = ZAux_DirectCommand(handle, "MOVE_DELAY(6000) AXIS(15)", ackbuff, 2048);//直接方式，在线命令
	//commandCheckHandler("MOVE_DELAY   ", ret);//添加move_dalay指令


	ret = ZAux_DirectCommand(handle, "MOVE_TABLE(7003, 1.000000) axis(15)", ackbuff, 2048);//MOVE_TABLE指令
	commandCheckHandler("MOVE_TABLE   ", ret);

	ret = ZAux_DirectCommand(handle, "MOVE_TABLE(7187,0.000000) axis(15)", ackbuff, 2048);
	commandCheckHandler("MOVE_TABLE   ", ret);


	ret = ZAux_DirectCommand(handle, "ZSMOOTH(15)=100", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("ZSMOOTH  ", ret);

	ret = ZAux_DirectCommand(handle, "FORCE_SPEED(15)=20", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("FORCE_SPEED  ", ret);


	ret = ZAux_DirectCommand(handle, "?*dpos", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("FORCE_SPEED  ", ret);
	printf("dpos=%s\n", ackbuff);
	ret = ZAux_DirectCommand(handle, "?*endmove", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("FORCE_SPEED  ", ret);
	printf("endmove=%s\n", ackbuff);


	ret = ZAux_DirectCommand(handle, "BASE(15,16,17,12,13,14,6,7,8,)\n MOVERV_LABS(-1.000000,906.722900,-111.649902,2239.583740,-3.667,27.206203,-176.7927,-0.001900,-0.004400,0.000000,)", ackbuff, 2048);//直接方式，在线命令
	commandCheckHandler("MOVERV_LABS   ", ret);

	ret = ZAux_DirectCommand(handle, "MOVE_TABLE(7003,2.000000) axis(15)", ackbuff, 2048);
	commandCheckHandler("MOVE_TABLE   ", ret);

	//ret=ZAux_DirectCommand(handle,"BASE(0,1,2,3,4,5,6)",ackbuff,2048);//直接方式，在线命令
	//ret=ZAux_DirectCommand(handle,"MOVERV_JABS(0,-20,20,0,90,0)",ackbuff,2048);//直接方式，在线命令
	//ret=ZAux_DirectCommand(handle,"BASE(0,1,2,3,4,5,6)\n MOVERV_JABS(0,-20,20,0,90,0)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_JABS   ", ret);

	//ret=ZAux_DirectCommand(handle,"base(15,16,17,12,13,14)",ackbuff,2048);//直接方式，在线命令
	//ret=ZAux_DirectCommand(handle,"moverv_l(-1, 200,0,0,0,0,0)",ackbuff,2048);//直接方式，在线命令
	//ret=ZAux_DirectCommand(handle,"base(15,16,17,12,13,14)\n moverv_l(-1, 200,0,0,0,0,0)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_l   ", ret);

	//ret=ZAux_DirectCommand(handle,"BASE(0,1,2,3,4,5,6)\n moverv_jabs(-10,16,7,1,70,0)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_JABS   ", ret);

	//ret=ZAux_DirectCommand(handle,"base(15,16,17,12,13,14)\n moverv_l(-1,0,100,0,0,10,10)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_l   ", ret);

	//ret=ZAux_DirectCommand(handle,"base(15,16,17,12,13,14)\n moverv_arc(-1, -16,-12,0,-20,-20,0,0,0,90)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_ARC   ", ret);

	//ret=ZAux_DirectCommand(handle,"base(15,16,17,12,13,14)\n moverv_p(-1,0,-100,0,0,-10,-10)",ackbuff,2048);//直接方式，在线命令
	//commandCheckHandler("MOVERV_P   ", ret);

	//运动类型获取}
	//Sleep(10000);
	ret = ZAux_Close(handle);
	handle = NULL;
	getchar();
	return 0;
}
*/

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
