#pragma once

#ifdef __cplusplus
extern "C" {
#endif


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

/**
 * @brief  本体坐标系的雅可比矩阵
 * @param  Blist       物体坐标系下的关节旋量
 * @param  thetalist   关节角数组
 * @param  num         关节数量
 * @param  J[out]      结果雅可比矩阵，转置后输出，使用前记得转置回来
 * @return             状态码
 */
int JacobianBody(
    const double Blist[][6],
    const double *thetalist,
    int num,
    double J[][6]
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
