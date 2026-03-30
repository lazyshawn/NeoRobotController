#pragma once

#include "task_plan/TaskTrajectory.h"

namespace task_plan {

/*
 * @brief 检查轨迹是否无碰撞
 * 
 * @param trajectory 输入轨迹
 * @return true 无碰撞
 * @return false 有碰撞
*/
bool check_trajectory_collision_free(const TaskTrajectory& trajectory);

/*
 * @brief 生成点到点的无碰撞路径
 * 
 * @param trajectory 输入轨迹
 * @return std::vector<TaskTrajectory> 无碰撞路径
*/
std::vector<TaskTrajectory>
generate_collision_free_trajectory(const TaskTrajectory& trajectory);

} // namespace task_plan
