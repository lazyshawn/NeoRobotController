
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "interpolation/InterpSegment.h"
#include "interpolation/InterpDispatch.h"

#include "AuxTransformation.h"

// 指令缓存
InterpBuffer interpBuffer;
// 调度器
InterpDispatcher dispatcher;
// 调度器状态
DispatcherState dispatcherState;
// 调度器输出信号
InterpSignalOut signalOut;
// 点位数据
PointInfo pointInfo;
MotionCfg motionCfg;
MoveCmd moveCmd;
// 当前关节角
PosData dpos;

// 插点线程
int push_trajectory();

int test_cuvre();

int test_dynamic();

int test_trans();

int main() {
	return test_trans();
	//return test_cuvre();
	//return test_dynamic();

	std::ofstream file, velFile;
	file.open("interp_pos.txt", std::ios::out);
	velFile.open("interp_vel.txt", std::ios::out);
	std::ofstream extFile, extVelFile;
	extFile.open("interp_ext.txt", std::ios::out);
	extVelFile.open("interp_ext_vel.txt", std::ios::out);

	dpos.pointType = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);
	dpos.extPos = std::vector<double>(6, 0.0);
	PosData dposPre = dpos;

	// 开启插点线程
	auto pointThreadWorker = std::thread(push_trajectory);

	// 执行插补线程
	bool interpBeg = false;
	for (int i=0; ; ++i) {
		// 传入当前角度
		dispatcherState.dpos = dpos;

		// 执行插补任务
		dispatcher.run_cycle_task(interpBuffer, signalOut, dispatcherState);

		// 更新当前角度
		dpos = dispatcherState.dpos;

		if (interpBeg) {
			// 位置输出
			file << dpos.rbtPos[0] << ", " << dpos.rbtPos[1] << ", " << dpos.rbtPos[2] << std::endl;
			// 速度输出
			for (int j = 0; j < 3; ++j) {
				dposPre.rbtPos[j] -= dpos.rbtPos[j];
			}
			velFile << sqrt(dposPre.rbtPos[0] * dposPre.rbtPos[0] + dposPre.rbtPos[1] * dposPre.rbtPos[1] + dposPre.rbtPos[2] * dposPre.rbtPos[2]) / interpBuffer.get_cycleTime() << std::endl;

			// 附加轴位置
			extFile << dpos.extPos[0] << std::endl;
			// 附加轴速度
			for (int j = 0; j < 3; ++j) {
				dposPre.extPos[j] -= dpos.extPos[j];
			}
			extVelFile << sqrt(dposPre.extPos[0] * dposPre.extPos[0] + dposPre.extPos[1] * dposPre.extPos[1] + dposPre.extPos[2] * dposPre.extPos[2]) / interpBuffer.get_cycleTime() << std::endl;

			dposPre = dpos;
		}

		if (dispatcherState.interpState % 2 == 1) {
			interpBeg = true;
		}
		else {
			interpBeg = false;
		}
	}

	// 线程回收
	if (pointThreadWorker.joinable()) {
		pointThreadWorker.join();
	}

	file.close();
	return 0;
}


int push_trajectory() {
	pointInfo.midPos.rbtPos = std::vector<double>(6, 0.0);
	pointInfo.endPos.rbtPos = std::vector<double>(6, 0.0);
	pointInfo.endPos.extPos = std::vector<double>(3, 0.0);
	motionCfg.moveType = 1;
	motionCfg.speed = 2;
	motionCfg.smooth = 40;

	SwingInterpParam swingCfg;
	swingCfg.enable = 1;
	swingCfg.freq = 1.0;
	swingCfg.leftWidth = swingCfg.rightWidth = 0.5;
	motionCfg.swingParam = swingCfg;

	// 点位指令插入缓存区
	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] += 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	motionCfg.moveType = 2;
	pointInfo.midPos.rbtPos[0] = pointInfo.endPos.rbtPos[0] + 8;
	pointInfo.midPos.rbtPos[1] = pointInfo.endPos.rbtPos[1] + 8;
	pointInfo.endPos.rbtPos[1] += 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	motionCfg.moveType = 1;
	pointInfo.endPos.rbtPos[0] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	while (!interpBuffer.buffer_ready()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	motionCfg.speed = 2;
	interpBuffer.add_move_point(pointInfo, motionCfg, moveCmd);

	// 开始信号使能
	dispatcher.interp_enable(true);

	return 0;
}

int test_cuvre() {
	std::ofstream file, velFile;
	file.open("interp_pos.txt", std::ios::out);
	velFile.open("interp_vel.txt", std::ios::out);

	DoubleSCurve curve;

	curve.set_constraint(10, 10);
	curve.set_condition(2, 0, 0,0);
	//curve.set_constraint(40, 10);
	//curve.set_condition(0, 9.2955883508964892, 4, 7.3523639345486407);
	curve.plan();

	double dt = 4e-3;
	int num = curve.get_duration() / dt;

	for (int i = 0; i < num + 1; ++i) {
		double curT = i * dt;
		double pos = curve.get_pos(curT);

		// 位置输出
		file << pos << ", " << 0 << ", " << 0 << std::endl;
		// 速度输出
		velFile << curve.get_vp() << std::endl;
	}

	return 0;
}

void tau(double* tau_out, const double* parms, const double* q, const double* dq, const double* ddq)
{
	double x0 = sin(q[1]);
	double x1 = -ddq[0];
	double x2 = cos(q[1]);
	double x3 = -dq[0];
	double x4 = x2 * x3;
	double x5 = dq[1] * x4 + x0 * x1;
	double x6 = dq[0] * dq[1] * x0 + x1 * x2;
	double x7 = x0 * x3;
	double x8 = dq[1] * parms[17] + parms[14] * x7 + parms[16] * x4;
	double x9 = dq[1] * parms[16] + parms[13] * x7 + parms[15] * x4;
	double x10 = -9.8100000000000005*x2;
	double x11 = dq[1] * parms[14] + parms[12] * x7 + parms[13] * x4;
	double x12 = -9.8100000000000005*x0;
	//
	tau_out[0] = ddq[0] * parms[5] + dq[0] * parms[10] + parms[11] * (((dq[0]) > 0) - ((dq[0]) < 0)) - x0 * (ddq[1] * parms[14] - dq[1] * x9 + parms[12] * x5 + parms[13] * x6 - parms[20] * x10 + x4 * x8) - x2 * (ddq[1] * parms[16] + dq[1] * x11 + parms[13] * x5 + parms[15] * x6 + parms[20] * x12 - x7 * x8);
	tau_out[1] = ddq[1] * parms[17] + dq[1] * parms[22] + parms[14] * x5 + parms[16] * x6 + parms[18] * x10 - parms[19] * x12 + parms[23] * (((dq[1]) > 0) - ((dq[1]) < 0)) - x11 * x4 + x7 * x9;
	//
	return;
}

int test_dynamic() {
	double tau_out[6] = { 0.0 }, q[6] = { 0.0 }, dq[6] = { 0.0 }, ddq[6] = { 0.0 };
	double param[90] = { 0.0 };

	// 关节0动力学参数
	int base = 0;
	// 惯性张量
	param[base + 0] = 0.031167987, param[base + 1] = 0.0, param[base + 2] = 0.0;
	param[base + 3] = 0.0, param[base + 4] = 0.03071271, param[base + 5] = -0.002236558;
	param[base + 6] = 0.0, param[base + 7] = -0.002236558, param[base + 8] = 0.005877402;
	// 质心位置
	param[base + 9] = 0.0, param[base + 10] = 0.0, param[base + 11] = 0.235842012;
	// m, fv, fc
	param[base + 12] = 3.167175036, param[base + 13] = 1.48e-3, param[base + 14] = 0.0;

	// 关节1动力学参数
	base = 15;
	param[base + 0] = 0.139827933, param[base + 1] = 0.000435835, param[base + 2] = 0.014816953;
	param[base + 3] = 0.000435835, param[base + 4] = 0.139767739, param[base + 5] = -0.002359708;
	param[base + 6] = 0.014816953, param[base + 7] = -0.002359708, param[base + 8] = 0.01264387;
	// 质心位置
	param[base + 9] = 0.016440414, param[base + 10] = -0.069406201, param[base + 11] = 0.183671051;
	// m, fv, fc
	param[base + 12] = 4.299222101, param[base + 13] = 0.817e-3, param[base + 14] = 0.0;

	// 关节2动力学参数
	base = 30;
	param[base + 0] = 0.022531203, param[base + 1] = -0.000450882, param[base + 2] = 0.004447562;
	param[base + 3] = -0.000450882, param[base + 4] = 0.023354298, param[base + 5] = -0.001591943;
	param[base + 6] = 0.004447562, param[base + 7] = -0.001591943, param[base + 8] = 0.004505884;
	// 质心位置
	param[base + 9] = 0.022215723, param[base + 10] = 0.009383099, param[base + 11] = 0.074628356;
	// m, fv, fc
	param[base + 12] = 2.248781033, param[base + 13] = 1.38e-3, param[base + 14] = 0.0;

	// 关节3动力学参数
	base = 45;
	param[base + 0] = 0.013319852, param[base + 1] = 0.0, param[base + 2] = 0.0;
	param[base + 3] = 0.0, param[base + 4] = 0.012519892, param[base + 5] = -0.00143894;
	param[base + 6] = 0.0, param[base + 7] = -0.00143894, param[base + 8] = 0.002788281;
	// 质心位置
	param[base + 9] = -0.000025487, param[base + 10] = -0.01092259, param[base + 11] = -0.071267109;
	// m, fv, fc
	param[base + 12] = 1.882223023, param[base + 13] = 71.2e-6, param[base + 14] = 0.0;

	// 关节4动力学参数
	base = 60;
	param[base + 0] = 0.005032625, param[base + 1] = 0.0, param[base + 2] = 0.0;
	param[base + 3] = 0.0, param[base + 4] = 0.002614418, param[base + 5] = -0.000331648;
	param[base + 6] = 0.0, param[base + 7] = -0.000331648, param[base + 8] = 0.003944803;
	// 质心位置
	param[base + 9] = 0.000002911, param[base + 10] = 0.029173125, param[base + 11] = 0.007018623;
	// m, fv, fc
	param[base + 12] = 1.507185922, param[base + 13] = 82.6e-6, param[base + 14] = 0.0;

	// 关节5动力学参数
	base = 75;
	param[base + 0] = 0.000997016, param[base + 1] = 0.0, param[base + 2] = 0.0;
	param[base + 3] = 0.0, param[base + 4] = 0.00100253, param[base + 5] = 0.0;
	param[base + 6] = 0.0, param[base + 7] = 0.0, param[base + 8] = 0.000447948;
	// 质心位置
	param[base + 9] = 0.000072631, param[base + 10] = 0.0, param[base + 11] = -0.044852927;
	// m, fv, fc
	param[base + 12] = 0.634034296, param[base + 13] = 36.7e-6, param[base + 14] = 0.0;

	tau(tau_out, param, q, dq, ddq);

	for (int i = 0; i < 6; ++i) {
		std::cout << tau_out[i] << ", ";
	}
	std::cout << std::endl;

	return 0;
}


int test_trans() {
	MatrixXd *Tsb = matrix_new_identity(4);
	MatrixXd *Tsc = matrix_new_identity(4);

	matrix_set(Tsb, 0, 0, cos(M_PI / 6));
	matrix_set(Tsb, 0, 1, -sin(M_PI / 6));
	matrix_set(Tsb, 1, 0, sin(M_PI / 6));
	matrix_set(Tsb, 1, 1, cos(M_PI / 6));
	matrix_set(Tsb, 0, 3, 1);
	matrix_set(Tsb, 1, 3, 2);
	//matrix_cout(Tsb);

	matrix_set(Tsc, 0, 0, cos(M_PI / 3));
	matrix_set(Tsc, 0, 1, -sin(M_PI / 3));
	matrix_set(Tsc, 1, 0, sin(M_PI / 3));
	matrix_set(Tsc, 1, 1, cos(M_PI / 3));
	matrix_set(Tsc, 0, 3, 2);
	matrix_set(Tsc, 1, 3, 1);
	//matrix_cout(Tsc);

	MatrixXd *T = matrix_new_identity(4);
	cTransInv(Tsb, T);
	//matrix_cout(T);

	matrix_multiply(Tsc, T, Tsb);
	//matrix_cout(Tsb);

	MatrixXd *se3 = matrix_new(4, 4, 0);
	cMatrixLog6(Tsb, se3);
	matrix_cout(se3);

	MatrixXd *V = matrix_new(6, 1, 0), *S = matrix_new(6, 1, 0);
	cSe3ToVec(se3, V);
	//matrix_cout(V);

	double theta = cAxisAng6(V, S);
	std::cout << "theta = " << theta << std::endl;
	matrix_cout(S);

	return 0;
}
