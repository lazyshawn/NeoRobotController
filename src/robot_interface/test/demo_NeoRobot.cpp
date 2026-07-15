
#include <windows.h>
#include<iostream>

#include "robot_interface/CoopRobotManager.h"

#include <Windows.h>
#include <DbgHelp.h>
#include <time.h>
#include <stdio.h>

void InvalidParameterHandler(const wchar_t* expression, const wchar_t* function,
	const wchar_t* file, unsigned int line, uintptr_t pReserved) {
	wprintf(L"Invalid parameter detected in function %s. File: %s Line: %u\n",
		function ? function : L"unknown",
		file ? file : L"unknown",
		line);

	*(int*)0 = 0;
}

LONG WINAPI CrashHandler(EXCEPTION_POINTERS* pException) {
	SYSTEMTIME st;
	GetLocalTime(&st);

	char dumpFileName[MAX_PATH];
	sprintf_s(dumpFileName, MAX_PATH, "crash_dump_%04d%02d%02d_%02d%02d%02d.dmp",
		st.wYear, st.wMonth, st.wDay,
		st.wHour, st.wMinute, st.wSecond);

	HANDLE hFile = CreateFileA(dumpFileName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		MINIDUMP_EXCEPTION_INFORMATION exceptionInfo;
		exceptionInfo.ThreadId = GetCurrentThreadId();
		exceptionInfo.ExceptionPointers = pException;
		exceptionInfo.ClientPointers = FALSE;

		BOOL success = MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(),
			hFile, MiniDumpWithFullMemory, &exceptionInfo, NULL, NULL);

		CloseHandle(hFile);

		if (success) {
			printf("Dump file created: %s\n", dumpFileName);
		}
		else {
			printf("Failed to create dump file. Error: %lu\n", GetLastError());
		}
	}
	else {
		printf("Failed to create file. Error: %lu\n", GetLastError());
	}

	return EXCEPTION_EXECUTE_HANDLER;
}

// 控制卡
//std::shared_ptr<FSAIRobotInterface::Controller> ZController(new FSAIRobotInterface::Controller);
FSAIRobotInterface::RobotGroupManager group;
// 轨迹
FSAIRobotInterface::DiscreteTrajectory trajList, trajList2;
//TrajectoryConfig trajCfg, trajCfg2;
std::vector<int> finish;


int main() {
	_set_invalid_parameter_handler(InvalidParameterHandler);
	SetUnhandledExceptionFilter(CrashHandler);

	//ZController->lazy_connect();
	//robot->set_ZController(ZController);

	group.new_robot(FSAIRobotInterface::RobotGroupManager::NEOROBOT);
	group.switch_auto(0, true);

	group.start_thread();

	// 轨迹
	FSAIRobotInterface::DiscreteTrajectory trajList;
	FSAIRobotInterface::SingleTrajectory curTraj;

	PosData dpos;
	dpos.pointType = 0;
	dpos.JntState = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);
	dpos.extPos = std::vector<double>(3, 0.0);

	// 点位数据
	PointInfo pointInfo;
	MotionCfg motionCfg;
	MoveCmd moveCmd;
	motionCfg.moveType = 1;
	motionCfg.speed = 2;
	motionCfg.smooth = 40;
	pointInfo.begPos = dpos;
	pointInfo.endPos = dpos;

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] += 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] += 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	group.push_new_trajectory(0, trajList);

	printf("Press <Enter> to exit.\n");
	getchar();

	return 0;

}
