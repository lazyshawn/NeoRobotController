
#include "interpolation/SegmentJoint.h"


/***********************************************************************
 *                        JointInterpSegment                           *
 ***********************************************************************/
 // 预处理
int JointInterpSegment::prehandle(InterpSegment& pre) {
	// - 把点位数据转化到关节空间

	// - 平滑处理
	// 当前段平滑设置
	procInfo.postSmooth = 0;
	// 前段平滑设置
	if (pre.motionCfg.smooth > 0) {
		pre.procInfo.postSmooth = pre.motionCfg.smooth;
		procInfo.preSmooth = pre.motionCfg.smooth;
	}

	// - 预处理完毕
	procInfo.processed = true;

	return 0;
}

// 规划
int JointInterpSegment::plan(InterpSegment& pre, InterpSegment& next) {

	// - 插补曲线规划
	// 当前段曲线规划
	curve.set_condition(pointInfo.begPos.rbtPos[0], pointInfo.endPos.rbtPos[0], 0, 0);
	curve.plan();

	double moveEndTime = 0.0;
	// 有平滑，且下一段轨迹已插入
	if (motionCfg.smooth > 0 && next.procInfo.processed) {
		// 下一段曲线规划
		DoubleSCurve nextCurve;
		nextCurve.set_condition(next.pointInfo.begPos.rbtPos[0], next.pointInfo.endPos.rbtPos[0], 0, 0);
		nextCurve.plan();

		// 当前段结束时间即平滑开始时间
		double smoothTime = std::min(nextCurve.get_Ta(), curve.get_Td()) * motionCfg.smooth * 1e-2;
		moveEndTime = curve.get_duration() - smoothTime;
	}
	else {
		moveEndTime = curve.get_duration();
	}

	// 有前平滑，保存前段插补曲线
	if (procInfo.preSmooth > 0) {
		curvePre.set_condition(pre.pointInfo.begPos.rbtPos[0], pre.pointInfo.endPos.rbtPos[0], 0, 0);
		curvePre.plan();
		curvePre.displacement(pre.get_current_time() - cycleTime, 1.0);
	}

	// 当前段规划时间
	procInfo.maxTime = moveEndTime;

	// 插补时间从零计数
	curTime = cycleTime;

	return 0;
}

// 插补
int JointInterpSegment::move(PosData& pos) {
	// 当前插补比例
	double ratio = curTime / procInfo.maxTime;

	pos.rbtPos = pointInfo.begPos.rbtPos;
	// 有前平滑，前段曲线参与插补
	if (procInfo.preSmooth > 0) {
		double prePos = curvePre.get_pos(curTime);
		double curPos = curve.get_pos(curTime);
		double curPos0 = curve.get_pos(0);
		pos.rbtPos[0] = curvePre.get_pos(curTime) + curve.get_pos(curTime) - curve.get_pos(0);
	}
	else {
		pos.rbtPos[0] = curve.get_pos(curTime);
	}

	// - 插补状态更新
	interpInfo.schedule = ratio;
	// 当前目标位置
	interpInfo.dpos = pos;

	// 插补时间累加
	curTime += cycleTime;

	// 返回插补进度
	return 0;
}

// 停止规划
int JointInterpSegment::stop_plan() {
	return 0;
}

// 重置
int JointInterpSegment::reset() {
	return 0;
}

// 获取当前时间
double JointInterpSegment::get_current_time() {
	return curTime;
}

