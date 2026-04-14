#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  6轴机械臂逆运动学配置结构体
 */
typedef struct {
	//! 关节类型
    int jntType[6];
    //! 关节方向
    double jntDir[6][3];
    //! 相邻关节坐标原点的相对坐标，原点可以是轴线上任意一点，通常按方便逆解计算的原则选取
    double jntAxisOffset[6][3];
	//! TCP 的零位姿态矩阵
	double M0[4][4];
	//! TCP 位姿
	double tcp[6];
	//! 外部轴旋量
	double ESlist[3][6];
	double EBlist[3][6];
    //! 奇异点阈值: 肩部，肘部，腕部
    double singThreshold[3];
} ArmKineConfig;

// 球腕关节带12轴相交
int GeoIK_3intersect_with_2intersect(const ArmKineConfig cfg, const double T[4][4], double theta[][6]);

// 球腕关节带23轴平行
int GeoIK_3intersect_with_2parallel(const ArmKineConfig cfg, const double T[4][4], double theta[][6]);

// 3组相邻平行关节带56轴相交
int GeoIK_3parallel_with_2intersect(const ArmKineConfig cfg, const double T[4][4], double theta[][6]);

#ifdef __cplusplus
}
#endif