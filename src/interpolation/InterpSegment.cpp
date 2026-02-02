
#include "interpolation/InterpSegment.h"


void PreProcessInfo::reset() {
	lineNum = -1;
	processed = false;
}

/***********************************************************************
 *                        InterpSegment                                *
 ***********************************************************************/
int InterpSegment::set_data(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	pointInfo = point;
	motionCfg = cfg;
	moveCmd = cmd;

	return 0;
}
