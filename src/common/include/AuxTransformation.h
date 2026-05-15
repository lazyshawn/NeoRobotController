#pragma once
/*
* @brief  正逆运动学辅助库
* @description  提供正逆运动学计算辅助函数
* @note    无
* @reference [Morden robotics: mechanics, planning, and control. Lynch and Park.]
*/

#include "AuxMatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
* @brief  计算旋转矩阵的逆
* @param  R     旋转矩阵
* @param  RInv  旋转矩阵的逆
* @return 0 成功，-1 失败
*/
SHARE_API_ int cRotInv(const MatrixXd *R, MatrixXd * RInv);

/*
* @brief  向量转换为so3矩阵
* @param  vec     向量
* @param  theta   旋转角度
* @param  so3  so3矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cVecToso3(const MatrixXd *omg, double theta, MatrixXd *so3);

/*
* @brief  so3矩阵转换为向量
* @param  so3     so3矩阵
* @param  vec  向量
* @return 0 成功，-1 失败
*/
SHARE_API_ int cSo3ToVec(const MatrixXd *so3, MatrixXd *vec);

/*
* @brief  计算旋转变换的指数坐标对应的轴角表示
* @param  exppc3  旋转变换的指数坐标
* @param  omghat  旋转轴的单位向量
* @return double 旋转角度
*/
SHARE_API_ double cAxisAng3(const MatrixXd *expc3, MatrixXd *omghat);

/*
* @brief  计算矩阵指数so(3)对应的旋转矩阵SO(3)
* @param  so3mat  矩阵指数so(3)
* @param  R       旋转矩阵SO(3)
* @return 0 成功，-1 失败
*/
SHARE_API_ int cMatrixExp3(const MatrixXd *so3mat, MatrixXd *R);

/*
* @brief  计算旋转矩阵SO(3)的指数坐标
* @param  R       旋转矩阵SO(3)
* @param  so3mat  矩阵指数的指数坐标so(3)
* @return 0 成功，-1 失败
*/
SHARE_API_ int cMatrixLog3(const MatrixXd *R, MatrixXd *so3mat);

/*
* @brief  计算旋转矩阵SO(3)和位移向量p对应的齐次变换矩阵T
* @param  R       旋转矩阵SO(3)
* @param  p       位移向量
* @param  T[out]  变换矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cRpToTrans(const MatrixXd *R, const MatrixXd *p, MatrixXd *T);

/*
* @brief  计算齐次变换矩阵T对应的旋转矩阵SO(3)和位移向量p
* @param  T       变换矩阵
* @param  R[out]  旋转矩阵SO(3)
* @param  p[out]  位移向量
* @return 0 成功，-1 失败
*/
SHARE_API_ int cTransToRp(const MatrixXd *T, MatrixXd *R, MatrixXd *p);

/*
* @brief  计算齐次变换矩阵T的逆
* @param  T          变换矩阵
* @param  TInv[out]  变换矩阵的逆
* @return 0 成功，-1 失败
*/
SHARE_API_ int cTransInv(const MatrixXd *T, MatrixXd *TInv);

/*
* @brief  计算6x1向量V对应的se(3)矩阵
* @param  V       6x1向量
* @param  theta   旋转角度
* @param  se3mat  se(3)矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cVecTose3(const MatrixXd *V, double theta, MatrixXd *se3mat);

/*
* @brief  计算se(3)矩阵对应的6x1向量V
* @param  se3mat  se(3)矩阵
* @param  V[out]  6x1向量
* @return 0 成功，-1 失败
*/
SHARE_API_ int cSe3ToVec(const MatrixXd *se3mat, MatrixXd *V);

/*
* @brief  计算齐次变换矩阵T的Adjoint矩阵AdT
* @param  T       变换矩阵
* @param  AdT[out]  Adjoint矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cAdjoint(const MatrixXd *T, MatrixXd *AdT);

/*
* @brief  根据，计算单位旋量
* @param  q       位置
* @param  s       旋转方向
* @param  h       螺距
* @param  infiniteH  是否为无限螺距，0为否，1为是
* @param  screw[out]  6x1向量
* @return 0 成功，-1 失败
*/
SHARE_API_ int cScrewToAxis(const MatrixXd *q, const MatrixXd *s, double h, int infiniteH, MatrixXd *screw);

/*
* @brief  计算空间变换的指数坐标的单位螺旋轴和旋转角度
* @param  expc6   指数坐标
* @param  S[out]  单位螺旋轴
* @return double 旋转角度
*/
SHARE_API_ double cAxisAng6(const MatrixXd *expc6, MatrixXd *S);

/*
* @brief  计算空间变换的指数坐标的齐次变换矩阵
* @param  se3mat  se(3)矩阵
* @param  T[out]  变换矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cMatrixExp6(const MatrixXd *se3mat, MatrixXd *T);

/*
* @brief  计算空间变换的指数坐标的se(3)矩阵
* @param  T            变换矩阵
* @param  se3mat[out]  se(3)矩阵
* @return 0 成功，-1 失败
*/
SHARE_API_ int cMatrixLog6(const MatrixXd *T, MatrixXd *se3mat);


#ifdef __cplusplus
}
#endif
