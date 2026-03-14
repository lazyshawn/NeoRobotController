
#include "interpolation/InterpSegmentBuffer.h"

// 矩阵计算辅助库
#include "AuxMatrix.h"

static const double dim_EPS = 1e-6;


/***********************************************************************
 *                        InterpBuffer                                 *
 ***********************************************************************/
InterpBuffer::InterpBuffer() {
	bufBeg = bufEnd = 0;
	maxBufNum = 10;
	// 保留一个缓冲位置，用于记录上一条运动
	reserveNum = 1;

	for (size_t i = 0; i < maxBufNum; ++i) {
		interpBuf.push_back(std::shared_ptr<InterpSegment>(new InterpSegment));
	}
}
InterpBuffer::~InterpBuffer() {

}

double InterpBuffer::get_cycleTime() {
	return cycleTime;
}

bool InterpBuffer::buffer_ready() {
	return (!bufOccupied) && (bufEnd - bufBeg - maxBufNum + reserveNum < 0);
}

int InterpBuffer::add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	// - 轨迹队列已满
	if (bufEnd - bufBeg - maxBufNum + reserveNum == 0) {
		printf("point buffer is full: %d\n", maxBufNum);
		return 1;
	}

	// 轨迹写入位置
	int curBuf = bufEnd, nextBuf, preBuf;
	get_neighbor_index(&preBuf, &bufEnd, &nextBuf);

	// 写入位置行号为非负，表示点位未执行完毕，不插入点位
	if (interpBuf[curBuf]->procInfo.lineNum >= 0) {
		return -1;
	}

	// - 插入轨迹
	bufOccupied = true;
	// 缓冲轨迹
	std::shared_ptr<InterpSegment> traj = std::shared_ptr<InterpSegment>(new InterpSegment);

	// 轨迹加入缓冲
	traj->set_data(point, cfg, cmd);
	interpBuf[curBuf] = traj;

	// - 预处理
	interpBuf[curBuf]->procInfo.procStage = 1;
	if (cfg.moveType == 0)
		joint_prehandle();
	else if(cfg.moveType == 1 || cfg.moveType == 2)
		cartesian_prehandle();
	interpBuf[curBuf]->procInfo.procStage = 2;

	// 行号
	interpBuf[curBuf]->procInfo.lineNum = bufEnd;

	// - 轨迹插入成功
	printf("add new point: %d\n", bufEnd);
	bufEnd++;
	bufOccupied = false;

	return 0;
}

int InterpBuffer::set_begin_pos(const PosData& begPos) {
	interpBuf[bufBeg % maxBufNum]->pointInfo.begPos = begPos;
	return 0;
}

int InterpBuffer::get_bufbeg() const {
	return bufBeg;
}

// 获取缓存个数
int InterpBuffer::get_buffer_size() const {
	return bufEnd - bufBeg;
}

// 获取当前目标位置
int InterpBuffer::get_cur_dpos(PosData& pos) const {
	int idx = bufBeg % maxBufNum;

	pos = interpBuf[idx]->interpInfo.dpos;

	return 0;
}

// 弹出队首轨迹
int InterpBuffer::pop_front() {
	int idx = bufBeg % maxBufNum;

	// 预处理信息清空
	interpBuf[idx]->procInfo.reset();

	bufBeg++;
	return 0;
}

// 获取指定行号的轨迹类型
InterpSegmentType InterpBuffer::get_segment_type(int bufNum) {
	InterpSegmentType type = InterpSegmentType::JOINT;
	return type;
}

// 获取相邻的轨迹索引 [0,N)
int InterpBuffer::get_neighbor_index(int *pre, int *cur, int *next) {
	*next = (*cur + 1) % maxBufNum;
	*pre = (*cur - 1) < 0 ? maxBufNum - 1 : *cur - 1;
	*cur = *cur % maxBufNum;

	return 0;
}
// 获取相邻的轨迹指针
int InterpBuffer::get_neighbor_buffer(int cur, InterpSegment *&preBuf, InterpSegment *&curBuf, InterpSegment *&nextBuf) {
	int next = (cur + 1) % maxBufNum;
	int pre = (cur - 1) < 0 ? maxBufNum - 1 : cur - 1;
	cur = cur % maxBufNum;

	preBuf = interpBuf[pre].get();
	curBuf = interpBuf[cur].get();
	nextBuf = interpBuf[next].get();

	return 0;
}

InterpSegment *InterpBuffer::get_following_buffer(int cur, int accent) {
	cur = (cur + accent) % maxBufNum;

	return interpBuf[cur].get();
}

// 执行轨迹插补
int InterpBuffer::move(PosData& pos) {
	int num = bufBeg, next, pre;
	get_neighbor_index(&pre, &num, &next);

	// 如果是第一条轨迹，等待若干周期，保证前两条轨迹平滑生效
	// 如果是新轨迹则进行规划, 预处理完成，未开始规划
	if (interpBuf[num]->procInfo.procStage == 2) {
		interpBuf[num]->procInfo.procStage = 3;
		if (interpBuf[num]->motionCfg.moveType == 0)
			joint_plane();
		else if (interpBuf[num]->motionCfg.moveType == 1)
			cartesian_plan();
		interpBuf[num]->procInfo.procStage = 4;
	}

	// - 执行插补
	if (interpBuf[num]->motionCfg.moveType == 0) {
		joint_move();
	}
	else if (interpBuf[num]->motionCfg.moveType == 1 || interpBuf[num]->motionCfg.moveType == 2) {
		cartesian_move();
	}

	// - 插补结果处理
	// 插补结果
	pos = interpBuf[num]->interpInfo.dpos;
	// 插补完成标识
	bool finish = interpBuf[num]->interpInfo.partId < 0;

	return finish;
}


int InterpBuffer::joint_prehandle() {
	// 预处理轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufEnd, preBuf, curBuf, nextBuf);

	// - 把点位数据转化到关节空间

	// - 平滑处理
	// 当前段平滑设置
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置
	if (preBuf->motionCfg.smooth > 0) {
		preBuf->procInfo.postSmooth = preBuf->motionCfg.smooth = curBuf->procInfo.preSmooth;
	}

	return 0;
}

int InterpBuffer::joint_plane() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// - 插补曲线规划
	// 当前段曲线规划
	curBuf->curve[0].set_condition(curBuf->pointInfo.begPos.rbtPos[0], curBuf->pointInfo.endPos.rbtPos[0], 0, 0);
	curBuf->curve[0].plan();

	double moveEndTime = 0.0;
	// 有后平滑，且下一段轨迹已插入
	if (curBuf->procInfo.postSmooth > 0 && nextBuf->procInfo.procStage == 2) {
		// 下一段曲线规划
		DoubleSCurve nextCurve;
		nextCurve.set_condition(nextBuf->pointInfo.begPos.rbtPos[0], nextBuf->pointInfo.endPos.rbtPos[0], 0, 0);
		nextCurve.plan();

		// 当前段结束时间即平滑开始时间
		double smoothTime = std::min(nextCurve.get_Ta(), curBuf->curve[0].get_Td()) * curBuf->motionCfg.smooth * 1e-2;
		moveEndTime = curBuf->curve[0].get_duration() - smoothTime;
	}
	else {
		moveEndTime = curBuf->curve[0].get_duration();
	}

	// 有前平滑，保存前段插补曲线
	if (curBuf->procInfo.preSmooth > 0) {
		curBuf->curvePre[0].set_condition(preBuf->pointInfo.begPos.rbtPos[0], preBuf->pointInfo.endPos.rbtPos[0], 0, 0);
		curBuf->curvePre[0].plan();
		curBuf->curvePre[0].displacement(preBuf->curTime - cycleTime, 1.0);
	}

	// 当前段规划时间
	curBuf->procInfo.maxTime = moveEndTime;

	// 插补时间从零计数
	curBuf->curTime = cycleTime;

	return 0;
}

int InterpBuffer::joint_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 插补结果
	PosData pos;
	// 当前插补比例
	double ratio = curBuf->curTime / curBuf->procInfo.maxTime;

	pos.rbtPos = curBuf->pointInfo.begPos.rbtPos;
	// 有前平滑，前段曲线参与插补
	if (curBuf->procInfo.preSmooth > 0) {
		double prePos = curBuf->curvePre[0].get_pos(curBuf->curTime);
		double curPos = curBuf->curve[0].get_pos(curBuf->curTime);
		double curPos0 = curBuf->curve[0].get_pos(0);
		pos.rbtPos[0] = curBuf->curvePre[0].get_pos(curBuf->curTime) + curBuf->curve[0].get_pos(curBuf->curTime) - curBuf->curve[0].get_pos(0);
	}
	else {
		pos.rbtPos[0] = curBuf->curve[0].get_pos(curBuf->curTime);
	}

	// - 插补状态更新
	// 运动比例
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;

	// 插补时间累加
	curBuf->curTime += cycleTime;

	// 返回插补进度
	return 0;
}

int InterpBuffer::cartesian_prehandle() {
	// 预处理轨迹，队尾开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufEnd, preBuf, curBuf, nextBuf);

	// --- 把点位数据转化到笛卡尔空间

	// --- 当前段计算轨迹长度
	// 直线轨迹
	if (curBuf->motionCfg.moveType == 1) {
		double det[3] = { 0 };
		for (int i = 0; i < 3; ++i) {
			det[i] = curBuf->pointInfo.endPos.rbtPos[i] - curBuf->pointInfo.begPos.rbtPos[i];
		}

		// 直线长度
		curBuf->procInfo.dist = std::sqrt(det[0] * det[0] + det[1] * det[1] + det[2] * det[2]);
		for (int i = 0; i < 3; ++i) {
			// 直线起点
			curBuf->procInfo.knot[i] = curBuf->pointInfo.begPos.rbtPos[i];
			// 直线方向
			curBuf->procInfo.dir[i] = det[i] / curBuf->procInfo.dist;
		}
	}
	// 圆弧轨迹
	else if (curBuf->motionCfg.moveType == 2) {
		double det[3] = { 0 };
	}

	// --- 平滑处理
	// 当前段平滑设置
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置，前段未开始规划，可能需要提前到前两条轨迹结束前若干个周期，以免当前段预处理时前一段正好开始规划
	if (preBuf->motionCfg.smooth > 0 && preBuf->procInfo.procStage == 2) {
		preBuf->procInfo.postSmooth = curBuf->procInfo.preSmooth = preBuf->motionCfg.smooth;
	}

	// --- 计算当前段前平滑控制点，前一段后平滑控制点
	if (curBuf->procInfo.preSmooth > 0) {
		double distPre = preBuf->procInfo.dist * 0.5;
		double distCur = curBuf->procInfo.dist * 0.5;
		// 平滑距离
		double smoothDist = std::min(distPre, distCur) * curBuf->procInfo.preSmooth * 1e-2;
		// 重复点，平滑距离为0，取消平滑
		if (smoothDist < dim_EPS) {
			curBuf->procInfo.preSmooth = 0.0;
			preBuf->procInfo.postSmooth = 0.0;

			curBuf->procInfo.preSmoothK = 0.0;
			preBuf->procInfo.postSmoothK = 1.0;
		}
		else {
			// 当前段前平滑
			curBuf->procInfo.preSmoothK = smoothDist / curBuf->procInfo.dist;
			// 前段后平滑
			preBuf->procInfo.postSmoothK = 1.0 - smoothDist / preBuf->procInfo.dist;

			// 直线拐点位置
			MatrixXd *corner = matrix_from_array(3, 1, curBuf->pointInfo.begPos.rbtPos.data(), 3);
			// 始末点切线方向
			MatrixXd *preDir = matrix_from_array(3, 1, preBuf->procInfo.dir, 3);
			MatrixXd *curDir = matrix_from_array(3, 1, curBuf->procInfo.dir, 3);
			// 计算当前段前平滑控制点
			MatrixXd *ctrl = matrix_copy(corner);
			double detK = 1.0 / 3;
			for (int i = 0; i < 3; ++i) {
				// 前半段 (u: 0 -> 0.5)
				matrix_plus(1, corner, -(1.0 - detK * i) * smoothDist, preDir, ctrl);
				matrix_to_array(ctrl, curBuf->procInfo.preCtrlPnt[i], 3);
				// 后半段 (u: 1 -> 0.5)
				matrix_plus(1, corner, (1.0 - detK * i) * smoothDist, curDir, ctrl);
				matrix_to_array(ctrl, curBuf->procInfo.preCtrlPnt[5 - i], 3);
			}
			// 前段后平滑控制点
			memcpy(preBuf->procInfo.postCtrlPnt, curBuf->procInfo.preCtrlPnt, 18 * sizeof(double));

			matrix_delete(preDir);
			matrix_delete(curDir);
			matrix_delete(ctrl);
		}
	}

	// --- 计算规划段长度
	if (curBuf->procInfo.preSmooth > 0) {
		// 前平滑曲线长度 (前平滑曲线长度在前平滑曲线插补完后重新计算，因为会有一定的误差，影响过渡到直线的速度计算)
		curBuf->procInfo.preBlendDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, 0.5, 1, 1000);
	}
	else {
		curBuf->procInfo.preBlendDist = 0.0;
	}
	// 无平滑段长度
	curBuf->procInfo.mainDist = curBuf->procInfo.dist * (1.0 - curBuf->procInfo.preSmoothK);
	curBuf->procInfo.postBlendDist = 0.0;
	// 实际前平滑开始位置，在前平滑曲线插补完成后需要重新计算，使用实际运动距离，否则影响切换到直线时的速度
	curBuf->procInfo.segmBegDist = curBuf->procInfo.preBlendDist;
	curBuf->procInfo.segmEndDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist;

	// --- 前一段曲线参数, 预处理完成，未开始规划
	if (preBuf->procInfo.procStage == 2) {
		// 后平滑曲线长度
		preBuf->procInfo.postBlendDist = curBuf->procInfo.preBlendDist;
		preBuf->procInfo.mainDist = preBuf->procInfo.dist * (preBuf->procInfo.postSmoothK - preBuf->procInfo.preSmoothK);
		// 实际后平滑开始位置，前平滑曲线插补完成后同步偏移
		preBuf->procInfo.segmEndDist = preBuf->procInfo.preBlendDist + preBuf->procInfo.mainDist;

		// 前一段轨迹的终点约束速度
		preBuf->procInfo.constrainedVel = std::min(preBuf->motionCfg.speed, curBuf->motionCfg.speed);
	}

	return 0;
}

int InterpBuffer::cartesian_plan() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// --- 速度规划
	// 设置位置规划约束
	curBuf->curve[0].set_constraint(curBuf->motionCfg.speed, 10);
	double vs = preBuf->procInfo.constrainedVel;
	// 前瞻回溯
	double ve = cartesian_look_ahead();
	curBuf->procInfo.constrainedVel = ve;
	
	// --- 插补曲线规划
	// 当前段规划总长度
	double planDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist;
	// 上一段剩余距离合并到前平滑段
	if (curBuf->procInfo.preSmooth > 0) {
		planDist += preBuf->procInfo.remainS;
	}
	curBuf->curve[0].set_condition(0, planDist, vs, ve);
	curBuf->curve[0].plan();

	// 计算整数倍插补周期后的剩余距离
	curBuf->procInfo.remainS = curBuf->curve[0].get_remain_dist(cycleTime);
	curBuf->procInfo.remainT = curBuf->curve[0].get_remain_time(cycleTime);

	// 当前段规划时间
	curBuf->procInfo.maxTime = curBuf->curve[0].get_duration();

	// --- 有前平滑时，获取上一段规划参数
	if (curBuf->procInfo.preSmooth > 0) {
		// 无平滑段始末位置
		curBuf->procInfo.segmBegDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->procInfo.doneU, 1.0, 1000);
		curBuf->procInfo.segmEndDist = curBuf->procInfo.segmBegDist + curBuf->procInfo.mainDist;

		// 位置分量不能合并插补，规划段右移，先插补上一条轨迹未完成的部分
		int shiftNum = ((preBuf->curve[0].get_duration() - preBuf->curve[0].get_offset()) - (preBuf->curTime - cycleTime)) / cycleTime;
		double shiftTime = shiftNum * cycleTime;
		// 右移整数个周期
		curBuf->curve[0].displacement(-shiftTime, 1);

		// 前一条轨迹规划曲线左移: 前平滑大于零开始(curve.offset) -> 后平滑从零开始(curTime)
		preBuf->curve[0].displacement((preBuf->curTime - cycleTime) + preBuf->curve[0].get_offset(), 1);
		// 保留时间清零
		//preBuf->curve.set_reserve_time(0);

		// 前一条轨迹附加轴规划左移
		preBuf->curve[6].displacement(preBuf->curTime - cycleTime, 1);
	}
	else {
		// 主运动状态初始化
		interpStatus.vel = 0.0;
		for (int i = 0; i < 3; ++i) {
			interpStatus.tan[i] = 0.0;
			interpStatus.cPos[i] = curBuf->pointInfo.begPos.rbtPos[i];
		}
	}

	// 有后平滑
	if (curBuf->procInfo.postSmooth > 0) {
		// 当前轨迹保留不足一个周期的部分，在下一条轨迹内插补
		curBuf->curve[0].set_reserve_time(curBuf->procInfo.remainT);
	}

	// --- 附加轴规划
	curBuf->curve[6].set_condition(curBuf->pointInfo.begPos.extPos[0], curBuf->pointInfo.endPos.extPos[0], 0, 0);
	double time1 = - curBuf->curve[0].get_offset();
	double time2 = curBuf->curve[0].calc_time_PiTPe(curBuf->procInfo.postBlendDist);
	// 附加轴规划段的位移时间
	double eulerTime = curBuf->curve[0].get_duration() - curBuf->curve[0].get_offset();
	// 姿态加速时间
	double oriAccT = 0.0;
	// 无平滑: 姿态匀速时长 = 位置匀速时长
	if (curBuf->procInfo.preSmooth < dim_EPS && curBuf->procInfo.postSmooth < dim_EPS)
		oriAccT = (curBuf->curve[0].get_duration() - curBuf->curve[0].get_Tv()) / 2;
	// 有前后平滑
	else if (curBuf->procInfo.preSmooth > dim_EPS && curBuf->procInfo.postSmooth > dim_EPS)
		oriAccT = time1;
	// 只有前平滑
	else if (curBuf->procInfo.preSmooth > dim_EPS && curBuf->procInfo.postSmooth < dim_EPS)
		oriAccT = 1.5 * time1;
	// 只有后平滑
	else if (curBuf->procInfo.preSmooth < dim_EPS && curBuf->procInfo.postSmooth > dim_EPS)
		oriAccT = 1.5 * time2;
	curBuf->curve[6].plan_by_duration(eulerTime, oriAccT, oriAccT / 2);

	// --- 摆焊规划
	SwingInterpParam swing;
	curBuf->moveCmd.get_swing(swing);
	// 有前平滑时，继承前段的规划，
	if (curBuf->procInfo.preSmooth > 0) {
		// --- 继承摆焊参数
		curBuf->curve[2] = preBuf->curve[2];

		// 修改摆焊参数，继承摆焊时间，当前摆焊目标位置
		SwingInterpParam preSwing;
		preSwing.deserialize(taskParam.swing);
		swing.state = preSwing.state;
		swing.time = preSwing.time;
		swing.pos = preSwing.pos;
	}
	else {
		swing.state = 0;
		swing.time = 0.0;
		swing.pos = 0.0;
	}
	// 后平滑决定摆焊运动时间
	if (curBuf->procInfo.postSmooth > 0) {
		swing.duration = 2 * (curBuf->curve[0].get_duration() - curBuf->curve[0].get_offset());
	}
	else {
		swing.duration = curBuf->curve[0].get_duration() - curBuf->curve[0].get_offset();
	}
	swing.serialize(taskParam.swing);

	// --- 插补状态复位
	// 插补完成标志复位
	curBuf->interpInfo.partId = 0;

	// 插补时间重新计数
	curBuf->curTime = cycleTime;

	return 0;
}

int InterpBuffer::cartesian_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);
	// 插补结果
	PosData pos;
	
	// --- 计算上一段+当前段的总位移
	// 上一段规划位移
	double preS = preBuf->procInfo.doneS;
	bool preDone = true;
	if (curBuf->procInfo.preSmooth > 0) {
		preS = preBuf->curve[0].get_pos(curBuf->curTime);
		preDone = preBuf->curve[0].done();
	}
	// 当前段规划位移
	double curS = curBuf->curve[0].get_pos(curBuf->curTime);
	bool curDone = curBuf->curve[0].done();
	// 当前总位移
	double moveS = preS - preBuf->procInfo.doneS;
	// 上一条轨迹插补完成
	if (preDone) {
		moveS += curS;
	}
	// 当前周期位移增量
	double detS = moveS - curBuf->interpInfo.curMoveS;

	// 当前插补比例
	double ratio = curS / (curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist);

	// --- 计算当前位置
	pos.rbtPos = curBuf->pointInfo.begPos.rbtPos;
	// 当前曲线参数，离开平滑段后复位
	double curU = 0.0;
	// 前平滑段
	if (curBuf->procInfo.preSmooth > 0 && moveS < curBuf->procInfo.segmBegDist) {
		if (curBuf->interpInfo.partId != 1) {
			curBuf->interpInfo.partId = 1;
		}
		double curPos[3];
		curU = bezier_interp(5, curBuf->procInfo.preCtrlPnt, curBuf->interpInfo.curU, detS);
		bezier_positioin(5, curBuf->procInfo.preCtrlPnt, curU, curPos);

		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = curPos[i];
		}
	}
	else {
		// 前平滑段结束后，基于实际走的S和曲线计算的S偏差重新计算 `前平滑段长度`，这样切换到直线段时速度计算才准确
		if (curBuf->interpInfo.partId == 1 && curBuf->procInfo.preSmooth > 0) {
			// 实际直线计算是从距离大于curMoveS开始的，计算的曲线距离(realDist) -> 实际规划距离(interpInfo.curMoveS)
			double realDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->procInfo.doneU, curBuf->interpInfo.curU, 1000);
			// 避免直线段计算的距离为负数
			double maxError = moveS - curBuf->procInfo.segmBegDist;

			// 切换到直线段速度不突变: 规划比实际多走的距离，计算直线时起点往后偏移即可补偿回来，但是实际终点位置会超出给定终点位置
			double error = curBuf->interpInfo.curMoveS - realDist;
			// 保证moveS走完后正好停在结束点: moveS = (preS - pre.doneS) + pre.remainS + preBlendDist + mainDist
			error = preBuf->procInfo.remainS + (preS - preBuf->procInfo.doneS) + curBuf->procInfo.preBlendDist - curBuf->procInfo.segmBegDist;

			curBuf->procInfo.segmBegDist += error;
			curBuf->procInfo.segmEndDist += error;
		}

		// 后平滑段
		if (curBuf->procInfo.postSmooth > 0 && moveS > curBuf->procInfo.segmEndDist) {
			// 后平滑段上的位移增量
			double dis = detS;

			if (curBuf->interpInfo.partId != 3) {
				curBuf->interpInfo.partId = 3;
				curBuf->interpInfo.curU = 0;
				//dis = moveS - curBuf->procInfo.segmBegDist - curBuf->procInfo.mainDist;
				// 减去直线段剩余长度
				dis -= curBuf->procInfo.segmEndDist - curBuf->interpInfo.curMoveS;
			}

			double curPos[3];
			curU = bezier_interp(5, curBuf->procInfo.postCtrlPnt, curBuf->interpInfo.curU, dis);
			bezier_positioin(5, curBuf->procInfo.postCtrlPnt, curU, curPos);

			for (int i = 0; i < 3; ++i) {
				pos.rbtPos[i] = curPos[i];
			}

			// 后平滑输出第一个点后结束当前段插补，剩下的部分再下一条轨迹中插补
			curDone = true;
		}
		// 无平滑段
		else {
			// 直线段长度
			double dis = moveS - curBuf->procInfo.segmBegDist + curBuf->procInfo.preSmoothK * curBuf->procInfo.dist;
			double lambda = (curBuf->procInfo.dist < dim_EPS) ? 0.0 : dis / curBuf->procInfo.dist;

			if (curBuf->interpInfo.partId != 2) {
				curBuf->interpInfo.partId = 2;
			}

			for (int i = 0; i < 3; ++i) {
				pos.rbtPos[i] = curBuf->pointInfo.begPos.rbtPos[i] * (1.0 - lambda) + curBuf->pointInfo.endPos.rbtPos[i] * lambda;
			}
		}
	}

	// --- 附加轴插补
	pos.extPos = std::vector<double>(6, 0.0);
	if (curBuf->procInfo.preSmooth > 0) {
		pos.extPos[0] = preBuf->curve[6].get_pos(curBuf->curTime) + curBuf->curve[6].get_pos(curBuf->curTime) - curBuf->curve[6].get_pos(0);
	}
	else {
		pos.extPos[0] = curBuf->curve[6].get_pos(curBuf->curTime);
	}

	// --- 摆焊叠加
	double swingAdd = 0.0;
	SwingInterpParam swing;
	swing.deserialize(taskParam.swing);
	// 半个摆动周期的时间
	double singleSwingTime = 0.5 / swing.freq;
	if (swing.state == 0) {
		swing.state = 1;
		curBuf->curve[2].set_condition(swing.pos, swing.rightWidth, 0, 0);
		curBuf->curve[2].plan_by_duration(singleSwingTime, singleSwingTime/2, singleSwingTime/10);
		curBuf->curve[2].plan();
		swing.time = cycleTime;
		swing.pos = swing.rightWidth;
	}
	else {
		swingAdd = curBuf->curve[2].get_pos(swing.time);

		if (curBuf->curve[2].done()) {
			swing.state = swing.state % 2 + 1;

			// 剩余时间不足一个完整周期
			double remainSwingTime = swing.duration - curBuf->curTime;
			bool timeReset = remainSwingTime < singleSwingTime * 2;
			if (timeReset)
				singleSwingTime = remainSwingTime;

			if (swing.state == 2) {
				curBuf->curve[2].set_condition(swing.pos, timeReset ? 0 : -swing.leftWidth, 0, 0);
				swing.pos = -swing.leftWidth;
			}
			else {
				curBuf->curve[2].set_condition(swing.pos, timeReset ? 0 : swing.rightWidth, 0, 0);
				swing.pos = swing.rightWidth;
			}

			curBuf->curve[2].plan_by_duration(singleSwingTime, singleSwingTime / 2, singleSwingTime / 10);
			swing.time = 0.0;
		}
		swing.time += cycleTime;
	}
	swing.serialize(taskParam.swing);

	// 实时状态
	interpStatus.vel = 0.0;
	for (int i = 0; i < 3; ++i) {
		interpStatus.tan[i] = pos.rbtPos[i] - interpStatus.cPos[i];
		interpStatus.cPos[i] = pos.rbtPos[i];
		interpStatus.vel += interpStatus.tan[i] * interpStatus.tan[i];
	}
	interpStatus.vel = sqrt(interpStatus.vel);
	for (int i = 0; i < 3; ++i) {
		interpStatus.tan[i] /= interpStatus.vel;
	}
	// 叠加到插补坐标系的Y方向
	pos.rbtPos[0] += swingAdd * interpStatus.tan[1];
	pos.rbtPos[1] -= swingAdd * interpStatus.tan[0];

	// --- 插补状态更新
	// 插补进度
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;
	curBuf->interpInfo.curMoveS = moveS;
	curBuf->interpInfo.curU = curU;

	// 插补完成: 有平滑时，剩余规划时间小于一个插补周期，将剩余距离移到下条轨迹前平滑段内插补
	if (curDone) {
		curBuf->interpInfo.partId = -1;
		// 下一段平滑从当前后平滑继续
		nextBuf->interpInfo.curU = curBuf->interpInfo.curU;
		curBuf->procInfo.doneS = curS;
		curBuf->procInfo.doneU = curU;
	}

	// 插补时间累加
	curBuf->curTime += cycleTime;

	return 0;
}

double InterpBuffer::cartesian_look_ahead() {
	// 1. 记录前瞻减速点: 从当前点可以正常运动加速到目标点
	// 2. 记录后溯减速点: 从当前点可以正常运动并停在结束点
	// 3. 合并减速点，得到最终减速点的速度
	// 4. 基于第一个减速点，计算到达当前段后平滑曲率点的终点速度
	// 5. 基于当前段/下一段设置速度和自适应速度约束Ve值

	// 轨迹前瞻，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 最大前瞻段数, 实际前瞻段数: forwardNum = 2表示除了当前段，还有两段需要前瞻
	int maxForwardNum = 10, forwardNum = 0;
	// 当前段有后平滑，启用前瞻
	if (curBuf->procInfo.postSmooth > 0) {
		for (int i = 0; i < maxForwardNum; ++i) {
			forwardNum++;

			InterpSegment *tmpBuf = get_following_buffer(bufBeg, i+1);
			if (tmpBuf->procInfo.postSmooth < dim_EPS)
				break;
		}
	}
	// 无需前瞻, 结束点速度为0
	if (forwardNum == 0)
		return 0.0;

	DoubleSCurve curve;
	// 前瞻/回溯段起点速度
	double vsForward = preBuf->procInfo.constrainedVel;
	// 第i段终点限速，当前段起点和最后一段终点速度(0)均已知
	std::vector<double> vlim(forwardNum, 0.0);

	// --- 前瞻: 从当前点加速
	for (int i = 0; i < forwardNum; ++i) {
		InterpSegment *tmpBuf = get_following_buffer(bufBeg, i);
		double dist = (i == 0) ? preBuf->procInfo.remainS : 0.0;
		dist += tmpBuf->procInfo.preBlendDist + tmpBuf->procInfo.mainDist + tmpBuf->procInfo.postBlendDist;

		// 最大提速速度
		curve.set_constraint(tmpBuf->motionCfg.speed, 10);
		curve.set_condition(0, dist, vsForward, 0);
		double maxSpeed = curve.get_max_speed(dist);

		// 前瞻约束速度
		vlim[i] = std::min(maxSpeed, tmpBuf->procInfo.constrainedVel);
		vsForward = vlim[i];
	}

	// --- 回溯: 从终点加速, i+1 段起点速度即为 i 段终点速度
	vsForward = 0.0;
	for (int i = forwardNum - 1; i >= 0; --i) {
		InterpSegment *tmpBuf = get_following_buffer(bufBeg, i + 1);
		double dist = tmpBuf->procInfo.preBlendDist + tmpBuf->procInfo.mainDist + tmpBuf->procInfo.postBlendDist;

		// 最大提速速度
		curve.set_constraint(tmpBuf->motionCfg.speed, 10);
		curve.set_condition(0, dist, vsForward, 0);
		double maxSpeed = curve.get_max_speed(dist);

		// 回溯约束速度
		vlim[i] = std::min(maxSpeed, vlim[i]);
		vsForward = vlim[i];
	}

	return vlim[0];
}
