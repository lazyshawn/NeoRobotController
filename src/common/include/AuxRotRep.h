#pragma once
/*****************************************************
 * @brief  Representations of Rotations
 *
 * 该文件中的姿态表示如下:
 * 旋转矩阵: 
 * R[9] = [RT_1, RT_2, RT_3], 其中 RT_i 为旋转矩阵转置后的第 i 列，即原矩阵的第 i 行
 * 欧拉角：
 * ZYX[3] - R(a,b,c) = I R(z,a) R(y,b) R(x,c)
 * RPY[3] - R(a,b,c) = R(z,a) R(y,b) R(x,c) I
 * 四元数:
 * q[4] = [w, x, y, z], 作为返回值时取 w 为正的解
 *****************************************************/

#ifdef __cplusplus
extern "C" {
#endif

// ZYX欧拉角转旋转矩阵
int Euler2Rot(const double euler[3], double R[9]);

// 旋转矩阵转欧拉角
int Rot2Euler(const double R[9], double euler[3]);

// 四元数转旋转矩阵
int Quat2Rot(const double q[4], double R[9]);

// 旋转矩阵转四元数
int Rot2Quat(const double R[9], double q[4]);

// 四元数球面距离
double QuatDistance(const double q0[4], const double q1[4]);

// 四元数球面插补
int QuatSlerp(const double q0[4], const double q1[4], double t, double q[4]);

#ifdef __cplusplus
}
#endif