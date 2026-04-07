#pragma once

#include "common/TrajectorySegment.h"

namespace task_plan {

// 任务空间轨迹类
class TaskTrajectory : public SegmentBase {
private:
    // 轨迹ID
    int trajId;
    // 轨迹类型: 0: T关节、F空间、1: T约束、F自由
    int trajType;
    // 任务类型: 空移、任务
    int taskType;

public:
    TaskTrajectory();
    ~TaskTrajectory();

};

} // namespace task_plan
