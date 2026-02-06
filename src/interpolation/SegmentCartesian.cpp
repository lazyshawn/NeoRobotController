
#include "interpolation/SegmentCartesian.h"

#include "AuxMatrix.h"

/***********************************************************************
 *                        CartesianInterpSegment                       *
 ***********************************************************************/
 // 预处理
int CartesianInterpSegment::prehandle(InterpSegment& pre) {
	// --- 把点位数据转化到笛卡尔空间

	// --- 当前段计算轨迹长度
	// 直线轨迹
	if ((motionCfg.moveType & 2) == 0) {
		double det[3] = { 0 };
		for (int i = 0; i < 3; ++i) {
			det[i] = pointInfo.endPos.rbtPos[i] - pointInfo.begPos.rbtPos[i];
		}

		// 直线长度
		procInfo.dist = std::sqrt(det[0] * det[0] + det[1] * det[1] + det[2] * det[2]);
		for (int i = 0; i < 3; ++i) {
			// 直线起点
			procInfo.knot[i] = pointInfo.begPos.rbtPos[i];
			// 直线方向
			procInfo.dir[i] = det[i] / procInfo.dist;
		}
	}
	// 圆弧轨迹
	else {

	}

	// --- 平滑处理
	// 当前段平滑设置
	procInfo.postSmooth = 0;
	// 前段平滑设置
	if (pre.motionCfg.smooth > 0) {
		pre.procInfo.postSmooth = pre.motionCfg.smooth;
		procInfo.preSmooth = pre.motionCfg.smooth;
	}

	// --- 计算当前段前平滑控制点，前一段后平滑控制点
	if (procInfo.preSmooth > 0) {
		double distPre = pre.procInfo.dist * 0.5;
		double distCur = procInfo.dist * 0.5;
		// 平滑距离
		double smoothDist = std::min(distPre, distCur) * procInfo.preSmooth * 1e-2;

		// 当前段前平滑
		procInfo.preSmoothK = smoothDist / procInfo.dist;
		// 前段后平滑
		pre.procInfo.postSmoothK = 1.0 - smoothDist / pre.procInfo.dist;

		// 直线拐点位置
		MatrixXd *corner = matrix_from_array(3, 1, pointInfo.begPos.rbtPos.data(), 3);
		// 始末点切线方向
		MatrixXd *preDir = matrix_from_array(3, 1, pre.procInfo.dir, 3);
		MatrixXd *curDir = matrix_from_array(3, 1, procInfo.dir, 3);
		// 计算当前段前平滑控制点
		MatrixXd *ctrl = matrix_copy(corner);
		double detK = 1.0 / 3;
		for (int i = 0; i < 3; ++i) {
			// 前半段 (u: 0 -> 0.5)
			matrix_plus(1, corner, -(1.0 - detK * i) * smoothDist, preDir, ctrl);
			matrix_to_array(ctrl, procInfo.preCtrlPnt[i], 3);
			// 后半段 (u: 1 -> 0.5)
			matrix_plus(1, corner, (1.0 - detK * i) * smoothDist, curDir, ctrl);
			matrix_to_array(ctrl, procInfo.preCtrlPnt[5 - i], 3);
		}
		// 前段后平滑控制点
		memcpy(pre.procInfo.postCtrlPnt, procInfo.preCtrlPnt, 18 * sizeof(double));

		matrix_delete(preDir);
		matrix_delete(curDir);
		matrix_delete(ctrl);
	}

	// --- 计算规划段长度
	// 前平滑曲线长度 (前平滑曲线长度在前平滑曲线插补完后重新计算，因为会有一定的误差，影响过渡到直线的速度计算)
	procInfo.preBlendDist = bezier_dist(5, procInfo.preCtrlPnt, 0.5, 1, 1000);
	// 无平滑段长度
	procInfo.mainDist = procInfo.dist * (1.0 - procInfo.preSmoothK);
	procInfo.postBlendDist = 0.0;

	// 后平滑曲线长度
	pre.procInfo.postBlendDist = procInfo.preBlendDist;
	pre.procInfo.mainDist = pre.procInfo.dist * (pre.procInfo.postSmoothK - pre.procInfo.preSmoothK);

	// - 预处理完毕
	procInfo.processed = true;

	return 0;
}

// 规划
int CartesianInterpSegment::plan(InterpSegment& pre, InterpSegment& next) {

	// - 插补曲线规划
	// 当前段曲线规划
	curve.set_condition(0, procInfo.preBlendDist + procInfo.mainDist + procInfo.postBlendDist, 0, 0);
	curve.plan();

	double moveEndTime = 0.0;
	moveEndTime = curve.get_duration();

	// 有前平滑，保存前段插补曲线
	if (procInfo.preSmooth > 0) {
		curvePre.set_condition(pre.pointInfo.begPos.rbtPos[0], pre.pointInfo.endPos.rbtPos[0], 0, 0);
		curvePre.plan();
		curvePre.displacement(pre.get_current_time() - cycleTime, 1.0);
	}

	// 当前段规划时间
	procInfo.maxTime = moveEndTime;

	// - 插补状态复位
	// 插补完成标志复位
	interpInfo.finish = false;

	// 插补时间重新计数
	curTime = cycleTime;

	return 0;
}

// 插补
int CartesianInterpSegment::move(PosData& pos) {

	// 当前时间位移
	double curS = curve.get_pos(curTime);
	// 当前插补比例
	double ratio = curS / (procInfo.preBlendDist + procInfo.mainDist + procInfo.postBlendDist);

	// 当前曲线参数，离开平滑段后复位
	double curU = 0.0;
	// 计算当前位置
	pos.rbtPos = pointInfo.begPos.rbtPos;
	// 前平滑段
	if (procInfo.preSmooth > 0 && curS < procInfo.preBlendDist) {
		double curPos[3];
		curU = bezier_interp(5, procInfo.preCtrlPnt, interpInfo.curU, curS - interpInfo.curS);
		bezier_positioin(5, procInfo.preCtrlPnt, curU, curPos);

		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = curPos[i];
		}
	}
	// 后平滑段
	else if (procInfo.postSmooth > 0 && curS > procInfo.preBlendDist + procInfo.mainDist) {
		double dis = curS - procInfo.preBlendDist - procInfo.mainDist;

		double curPos[3];
		curU = bezier_interp(5, procInfo.postCtrlPnt, interpInfo.curU, curS - interpInfo.curS);
		bezier_positioin(5, procInfo.postCtrlPnt, curU, curPos);

		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = curPos[i];
		}
	}
	// 无平滑段
	else {
		//基于实际走的S和曲线计算的S偏差重新计算 `前平滑段长度`

		double dis = curS - procInfo.preBlendDist + procInfo.preSmoothK * procInfo.dist;
		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = pointInfo.begPos.rbtPos[i] * (1.0 - dis / procInfo.dist) + pointInfo.endPos.rbtPos[i] * dis / procInfo.dist;
		}
	}

	// - 插补状态更新
	// 插补进度
	interpInfo.schedule = ratio;
	// 当前目标位置
	interpInfo.dpos = pos;
	interpInfo.curS = curS;
	interpInfo.curU = curU;

	// 插补完成
	if (curTime > procInfo.maxTime) {
		interpInfo.finish = true;
		// 下一段从当前距离开始
	}
	//interpInfo.finish = (curTime > procInfo.maxTime);

	// 插补时间累加
	curTime += cycleTime;

	return 0;
}

// 停止规划
int CartesianInterpSegment::stop_plan() {
	return 0;
}

// 重置
int CartesianInterpSegment::reset() {
	return 0;
}

// 获取当前时间
double CartesianInterpSegment::get_current_time() {
	return curTime;
}


