#pragma once
/**
 * @brief  基于旋量方法的六轴机器人逆解的几何解法子问题
 *
 * @ref    [Paden-Kahan子问题](https://en.wikipedia.org/wiki/Paden-Kahan_subproblems)
 *         [Canonical Subproblems for Robot Inverse Kinematics](https://arxiv.org/pdf/2211.05737v2)
 */

 #ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Paden-Kahan subproblem 1
 * 
 * 点 p 绕 S 轴旋转到点 q，求解旋转角度 theta
 *
 * @param S 6x1 twist, 零螺距的单位旋量
 * @param p 3x1 vector, 起点位置
 * @param q 3x1 vector, 终点位置
 * @param theta 最小二乘解，1 组解
 * @return int 0 if success, -1 if fail
 */
int PKsubproblem_1(
    const double *S,
    const double *p,
    const double *q,
    double *theta
);

/**
 * @brief Paden-Kahan subproblem 2
 * 
 * 点 p 依次绕 S1 轴和 S2 轴旋转到点 q，求解旋转角度 theta1 和 theta2
 *
 * @param S1 6x1 twist, 零螺距的单位旋量
 * @param S2 6x1 twist, 零螺距的单位旋量
 * @param p 3x1 vector, 起点位置
 * @param q 3x1 vector, 终点位置
 * @param theta 最小二乘解，2 组解
 * @return int 0 if success, -1 if fail
 */
int PKsubproblem_2(
    const double *S1,
    const double *S2,
    const double *p,
    const double *q,
    double *theta
);

/**
 * @brief Paden-Kahan subproblem 3
 * 
 * 点 p 绕 S 轴旋转到点 p',使得点 p' 到点 q 的距离为 det，求解旋转角度 theta
 *
 * @param S 6x1 twist, 零螺距的单位旋量
 * @param p 3x1 vector, 起点位置
 * @param q 3x1 vector, 终点位置
 * @param det 旋转角度的确定性
 * @param theta 最小二乘解，2 组解
 * @return int 0 if success, -1 if fail
 */
int PKsubproblem_3(
    const double *S,
    const double *p,
    const double *q,
    double det,
    double *theta
);

/**
 * @brief Canonical subproblem 1
 * 
 * 点 p 绕 k 轴旋转到点 q，求解旋转角度 theta
 *
 * @param k 3x1 vector, 单位旋转方向
 * @param p 3x1 vector, 起点位置
 * @param q 3x1 vector, 终点位置
 * @param theta 最小二乘解，1 组解
 * @param num 解个数
 * @return int 0 精确解, 1 近似解, 小于 0 表示异常
 */
int canonical_subproblem_1(
    const double *pPos,
    const double *dir,
    const double *qPos,
    double *theta,
    int *num
);

int canonical_subproblem_2(
    const double *pos1,
    const double *dir1,
    const double *pos2,
    const double *dir2,
    double *theta1,
    double *theta2,
	int *num
);

int canonical_subproblem_3(
	const double *pos1,
	const double *dir1,
	const double *pos2,
	double dist,
	double *theta,
	int *num
);

/**
 * @brief Canonical subproblem 4
 * 
 * 点 p 绕 k 轴旋转到点 q，使得点 q 到以 h 为法向的平面的距离为 dist，求解旋转角度 theta
 *
 * @param k 3x1 vector, 单位旋转方向
 * @param p 3x1 vector, 起点位置
 * @param h 3x1 vector, 平面法向方向，不要求单位化
 * @param dist 旋转角度的确定性
 * @param theta 最小二乘解，2 组解
 * @param num 解个数
 * @return int 0 精确解, 1 近似解, 小于 0 表示异常
 */
int canonical_subproblem_4(
    const double *pos,
    const double *kDir,
    const double *hDir,
    double dist,
    double *theta,
    int *num
);

int canonical_subproblem_5(
    const double *pos1,
    const double *dir1,
    const double *shift1,
    const double *dir2,
    const double *pos3,
    const double *dir3,
    const double *shift3,
    double *theta1,
    double *theta2,
    double *theta3,
    int *num
);


#ifdef __cplusplus
}
#endif