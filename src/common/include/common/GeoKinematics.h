#pragma once
/*
 * @brief  公开的正逆运动学接口
*/

#include "common/ExportSharedAPI.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  6轴机械臂运动学配置结构体
 */
typedef struct {
    // --- 本体运动学配置
    //! 机器人构型: I3P2, I3I2, P3I2
    int robotConfigType;
    //! 关节类型
    int jntType[6];
    //! 关节方向
    double jntDir[6][3];
    //! 相邻关节坐标原点的相对坐标，原点可以是轴线上任意一点，通常按方便逆解计算的原则选取
    double jntAxisOffset[6][3];
    //! 零位偏移角
    double jntZeroOffset[6];
    //! 耦合比: 23,56
    double couple[2];
    //! 奇异点阈值: 肩部，肘部，腕部
    double singThreshold[3];
    //! 关节正限位
    double jntUpperLimit[6];
    //! 关节负限位
    double jntLowerLimit[6];

    // --- 系统运动学配置
	//! 世界坐标系零点位置
	double rbtZeroOffset[6];
	//! TCP 编号
	int tcpId;
	//! TCP 记录值，最多记录 10 组
	double tcp[10][6];
	//! 地轨方向
	double extDir[3];
	//! 地轨正限位
	double extUpperLimit[3];
	//! 地轨负限位
	double extLowerLimit[3];

	// --- 推导参数
	//! 空间坐标系的关节旋量: 通过关节类型、关节方向、关节轴线上任意一点计算得到
	double Slist[6][6];
	//! 物体坐标系的关节旋量: 通过空间坐标系的关节旋量和零位姿态矩阵计算得到
	double Blist[6][6];
    //! 法兰中心点位置
    double efc[3];
	//! TCP 的零位姿态矩阵
	double M0[4][4];
	//! 外部轴旋量
	double ESlist[3][6];
	double EBlist[3][6];
}GeoKineConfig;

typedef struct {
	//! 目标TCP位姿
	double T[4][4];
	//! 参考关节角，用于确定最终解
	double refTheta[6];
} GeoIKRequest;

typedef struct {
	//! 逆解集合
	double theta[8][6];
	//! 解索引: < 0 表示无解
	int id;
} GeoIKResponse;

typedef struct {
	//! 目标关节角度
	double *thetalist;
	//! 不同参考系求解: Tbt, Tbf, Twt, Twf
	int refFrame;
    //! 多TCP姿态求解
    int multiTcp;
} GeoFKRequest;

//! 初始化运动学配置, 根据基础配置计算并更新完整配置
SHARE_API_ int construct_GeoKineConfig(GeoKineConfig *cfg);

//! 正运动学计算
SHARE_API_ int GeoFK(const GeoKineConfig *cfg, const GeoFKRequest *req, double T[4][4]);

//! 逆运动学计算
SHARE_API_ int GeoIK(const GeoKineConfig *cfg, const GeoIKRequest *req, GeoIKResponse *resp);

//! 雅可比计算
SHARE_API_ int GeoJacobian(const GeoKineConfig *cfg, const double *joint, double J[][6]);

#ifdef __cplusplus
}
#endif