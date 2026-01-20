
#include "interpolation/InterpSegment.h"

std::shared_ptr<InterpBuffer> InterpSegment::interpBuf = std::make_shared<InterpBuffer>();


/***********************************************************************
 *                        JointInterpSegment                           *
 ***********************************************************************/
// 预处理
int JointInterpSegment::prehandle() {
	return 0;
}

// 规划
int JointInterpSegment::plan() {
	return 0;
}

// 插补
int JointInterpSegment::move() {
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
int JointInterpSegment::get_current_time() {
	return 0;
}
