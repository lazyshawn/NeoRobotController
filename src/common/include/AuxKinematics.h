#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  6轴机械臂运动学配置结构体
 */
struct ArmKineConfig {
    // --- 本体运动学配置
	//! 空间坐标系的关节旋量
    double Slist[6][6];
	//! 物体坐标系的关节旋量
	double Blist[6][6];
    //! 零位偏移角
    double jntZeroOffset[6];
    //! 耦合比: 23,56
    double couple[2];
    //! 奇异点阈值: 肩部，肘部，腕部
    double singThreshold[3];

    // --- 本体运动学配置
	//! 外部轴旋量
	double ESlist[3][6];
	double EBlist[3][6];
	//! 世界坐标系零点位置
	double rbtZeroOffset[6];
	//! TCP 编号
	int tcpId;
	//! TCP 记录值，最多记录 10 组
	double tcp[10][6];
};

/**
 * @brief  前向运动学
 * @param  M0          零位时的变换矩阵
 * @param  Blist       物体坐标系下的关节旋量
 * @param  thetalist   关节角数组
 * @param  num         关节数量
 * @param  T[out]      结果变换矩阵
 * @return             状态码
 */
int FKinBody(
    const double M0[4][4],
    const double Blist[][6],
    const double *thetalist,
    int num,
    double T[4][4]
);

int FKinSpace(
    const double M0[4][4],
    const double Slist[][6],
    const double *thetalist,
    int num,
    double T[4][4]
);

int JacobianBody(
    const double Blist[][6],
    const double *thetalist,
    int num,
    double J[6][4]
);

int JacobianSpace(
    const double Slist[][6],
    const double *thetalist,
    int num,
    double J[][6]
);

int IKinBody(
    const double Blist[][6],
    const double M0[4][4],
    const double T[4][4],
    const double *thetalist0,
    double emog,
    double ev,
    double *thetalist
);

int IKinSpace(
    const double Slist[][6],
    const double M0[4][4],
    const double T[4][4],
    const double *thetalist0,
    double emog,
    double ev,
    double *thetalist
);

#ifdef __cplusplus
}
#endif
