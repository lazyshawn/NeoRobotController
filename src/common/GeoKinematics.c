#include "common/GeoKinematics.h"

#include "AuxMatrix.h"
#include "AuxRotRep.h"
#include "GeoFK.h"
#include "GeoIK.h"

static const double dim_ESP = 1e-9;

int GeoCfg2ArmCfg (const GeoKineConfig *cfg, ArmKineConfig *armCfg) {
    MatrixXd *pos = matrix_new(3,1,0.0);
    MatrixXd *detP = matrix_new(3,1,0.0);
    MatrixXd *M0 = matrix_new_identity(4);

    // --- 1. 复制关节配置
    for (int i=0; i<6; ++i) {
        armCfg->jntType[i] = cfg->jntType[i];
        for (int j=0; j<3; ++j) {
            armCfg->jntDir[i][j] = cfg->jntDir[i][j];
            armCfg->jntAxisOffset[i][j] = cfg->jntAxisOffset[i][j];
        }
    }

	// --- 2. 复制 TCP 位姿
	for (int i = 0; i < 6; ++i) {
		armCfg->tcp[i] = cfg->tcp[cfg->tcpId][i];
	}

    // --- 3. 计算 TCP 的零位姿态矩阵
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			armCfg->M0[i][j] = cfg->M0[i][j];
		}
	}

    // --- 4. 复制外部轴旋量

    // --- 5. 复制奇异点阈值

    matrix_delete(pos);
    matrix_delete(detP);
    matrix_delete(M0);
    return 0;
}

// 根据写入的运动学配置，更新计算需要推导的值
int construct_GeoKineConfig(GeoKineConfig *cfg) {
    // 根据写入的运动学配置，更新计算需要推导的值
	MatrixXd *ws = matrix_new(3, 1, 0.0);
	MatrixXd *vs = matrix_new(3, 1, 0.0);
	MatrixXd *pos = matrix_new(3, 1, 0.0);
	MatrixXd *detP = matrix_new(3, 1, 0.0);
	MatrixXd *V = matrix_new(6, 1, 0.0);
	MatrixXd *M0 = matrix_new_identity(4);

    // --- 1. 计算6个关节的空间坐标旋量
	for (int i = 0; i < 6; ++i) {
		// 关节方向
		matrix_copy_array(ws, cfg->jntDir[i], 3);
		// 如果长度不为1则单位化
		if (fabs(matrix_squared_norm(ws) - 1.0) > dim_ESP) {
			matrix_normalize(ws);
			matrix_to_array(ws, cfg->jntDir[i], 3);
		}
		// 关节原点
		matrix_copy_array(detP, cfg->jntAxisOffset[i], 3);
		matrix_plus(1.0, pos, 1.0, detP, pos);
		// 速度分量: -(ws x pos)
		matrix_outer_product(pos, ws, vs);
		// 设置空间坐标旋量
		matrix_set_block(V, 0, 0, ws);
		matrix_set_block(V, 3, 0, vs);
		// 空间坐标旋量写入结构体
		matrix_to_array(V, cfg->Slist[i], 6);
	}
    // 法兰中心点位置
    matrix_to_array(pos, cfg->efc, 3);

	// --- 2. 计算 TCP 的零位姿态矩阵(正解需要)
	// TCP 位置
	matrix_copy_array(pos, cfg->efc, 3);
	matrix_copy_array(detP, cfg->tcp[cfg->tcpId], 3);
	matrix_plus(1.0, pos, 1.0, detP, pos);
	matrix_set_block(M0, 0, 3, pos);
	// TCP 姿态
	double euler[3], rot[9];
	for (int i = 0; i < 3; ++i) {
		euler[i] = cfg->tcp[cfg->tcpId][3 + i];
	}
	Euler2Rot(euler, rot);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			matrix_set(M0, i, j, rot[i * 3 + j]);
		}
	}
	// 赋值到 TCP 姿态矩阵
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			cfg->M0[i][j] = matrix_at(M0, i, j);
		}
	}

	matrix_delete(ws);
	matrix_delete(vs);
	matrix_delete(pos);
	matrix_delete(detP);
	matrix_delete(V);
	matrix_delete(M0);
    return 0;
}

//! 正运动学计算
int GeoFK(const GeoKineConfig *cfg, const GeoFKRequest *req, double T[4][4]) {
	FKinSpace(cfg->M0, cfg->Slist, req->thetalist, 6, T);
    return 0;
}

//! 雅可比计算
int GeoJacobian(const GeoKineConfig *cfg, const double *joint, double J[][6]) {
    return 0;
}

//! 逆运动学计算
int GeoIK(const GeoKineConfig *cfg, const GeoIKRequest *req, GeoIKResponse *resp) {
    // 提取逆解配置
    ArmKineConfig armCfg;
	GeoCfg2ArmCfg(cfg, &armCfg);

	// 选择逆解算法
	int solNum = 0;
	solNum = GeoIK_3intersect_with_2parallel(armCfg, req->T, resp->theta);

	// 筛选逆解
	resp->id = 0;

    return solNum;
}
