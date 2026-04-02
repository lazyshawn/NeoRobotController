
#include "AuxTransformation.h"
#include "AuxMatrix.h"

const double auxEPS = 1e-9;

// Computes the inverse of the rotation matrix R.
int cRotInv(const MatrixXd *R, MatrixXd * RInv) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            RInv->data[i*3+j] = R->data[j*3+i];
        }
    }
    return 0;
}

// Returns the 3x3 skew-symmetric matrix corresponding to omg.
int cVecToso3(const MatrixXd *omg, double theta, MatrixXd *so3) {
    // 下三角元素
    matrix_set(so3, 2, 1, omg->data[0] * theta);
    matrix_set(so3, 2, 0, -omg->data[1] * theta);
    matrix_set(so3, 1, 0, omg->data[2] * theta);

    // 剩余元素，对角线为0，上三角元素与下三角元素互为相反数
    for(int i=0; i<3; i++){
        for(int j=i; j<3; j++){
            double val = matrix_at(so3, j, i);
            matrix_set(so3, i, j, i==j? 0 : -val);
        }
    }

    return 0;
}

// Returns the 3-vector corresponding to the 3x3 skew-symmetric matrix so3mat.
int cSo3ToVec(const MatrixXd *so3, MatrixXd *vec) {
    vec->data[0] = (matrix_at(so3, 2, 1) - matrix_at(so3, 1, 2)) / 2.0;
    vec->data[1] = (matrix_at(so3, 0, 2) - matrix_at(so3, 2, 0)) / 2.0;
    vec->data[2] = (matrix_at(so3, 1, 0) - matrix_at(so3, 0, 1)) / 2.0;

    return 0;
}

// Extracts the unit rotation axis omghat and the rotation amount theta from the 3-vector omg of exponential coordinates for rotation, expc3.
double cAxisAng3(const MatrixXd *expc3, MatrixXd *omghat) {
    double theta = matrix_norm(expc3);
    if (theta == 0.0) {
        return 0.0;
    }
    for (int i=0; i<3; i++) {
        omghat->data[i] = expc3->data[i] / theta;
    }

    return theta;
}

// Computes the rotation matrix R SO(3) corresponding to the matrix exponential of so3mat so(3).
int cMatrixExp3(const MatrixXd *so3mat, MatrixXd *R) {
    MatrixXd *omghat = matrix_new(3, 1, 0);
    double theta = cAxisAng3(so3mat, omghat);

    // 罗德里格斯公式
    MatrixXd *wx = matrix_new(3,3,0);
    cVecToso3(omghat, 1.0, wx);
    // (1-cq) * wx^2
    matrix_multiply(wx, wx, R);
    matrix_scale(R, 1.0 - cos(theta));
    // sq * wx
    matrix_scale(wx, sin(theta));
    matrix_plus(1.0, R, 1.0, wx, R);
    // I
    matrix_set_identity(wx);
    matrix_plus(1.0, R, 1.0, wx, R);

    matrix_delete(omghat);
    matrix_delete(wx);

    return 0;
}

// Computes the matrix logarithm so3mat so(3) of the rotation matrix R SO(3).
int cMatrixLog3(const MatrixXd *R, MatrixXd *so3mat) {
    // R 为旋转矩阵(行列式为1)时，其迹tr(R) 的范围为 [-1, 3]
    double theta = 0.0;

    // Case 1. R is the identity matrix.
    double tr = 0.0;
    for (int i=0; i<3; i++) {
        tr += matrix_at(R, i, i);
    }

    // Case 1. R is the identity matrix.
    if (3.0 - tr < auxEPS) {
        matrix_set_all(so3mat, 0.0);
    }
    // Case 2. trR = -1. theta = pi, if omg is a solution, then so is -omg.
    else if (tr + 1.0 < auxEPS) {
        theta = M_PI;
        MatrixXd *omghat = matrix_new(3, 1, 0);

        double denom[3] = {0.0};
        int maxIdx = 0;
        for (int i=0; i<3; ++i) {
            denom[i] = 2 * (matrix_at(R, i, i) + 1.0);
            // 更新denom中最大元素的位置
            if (denom[i] > denom[maxIdx]) {
                maxIdx = i;
            }
        }

        for (int i=0; i<3; ++i) {
            omghat->data[i] = matrix_at(R, 0, maxIdx);
            if (i == maxIdx) {
                omghat->data[i] += 1.0;
            }
        }
        matrix_scale(omghat, 1.0 / sqrt(denom[maxIdx]));

        matrix_scale(omghat, theta);
        cVecToso3(omghat, 1.0, so3mat);
        matrix_delete(omghat);
    }
    // Case 3. general case.
    else {
        theta = acos((tr - 1.0) / 2.0);
        MatrixXd *RT = matrix_new(3,3,0.0);
        cRotInv(R, RT);
        matrix_plus(1.0, R, -1.0, RT, so3mat);
        matrix_scale(so3mat, theta / (2 * sin(theta)));

        matrix_delete(RT);
    }

    return 0;
}

// Builds the transformation matrix T corresponding to a rotation matrix R and a position vector p.
int cRpToTrans(const MatrixXd *R, const MatrixXd *p, MatrixXd *T) {
    matrix_set_identity(T);
    matrix_set_block(T, 0, 0, R);
    matrix_set_block(T, 0, 3, p);
    
    return 0;
}

// Extracts the rotation matrix and position vector from a homogeneous transformation matrix T.
int cTransToRp(const MatrixXd *T, MatrixXd *R, MatrixXd *p) {
    matrix_get_block(T, 0, 0, R);
    matrix_get_block(T, 0, 3, p);

    return 0;
}

// Computes the inverse of a homogeneous transformation matrix T.
int cTransInv(const MatrixXd *T, MatrixXd *TInv) {
    MatrixXd *R = matrix_new(3,3,0.0);
    MatrixXd *p = matrix_new(3,1,0.0);
    cTransToRp(T, R, p);

    MatrixXd *RT = matrix_new(3,3,0.0);
    cRotInv(R, RT);
    MatrixXd *pt = matrix_new(3,1,0.0);
    matrix_multiply(RT, p, pt);
    matrix_scale(pt, -1.0);

    cRpToTrans(RT, pt, TInv);
    matrix_delete(R);
    matrix_delete(p);
    matrix_delete(RT);
    matrix_delete(pt);

    return 0;
}

// Returns the se(3) matrix corresponding to a 6-vector twist V.
int cVecTose3(const MatrixXd *V, double theta, MatrixXd *se3mat) {
    matrix_set_all(se3mat, 0.0);

    // 旋转分量
    MatrixXd *vec = matrix_new(3,1,0);
    matrix_get_block(V, 0, 0, vec);
    MatrixXd *wx = matrix_new(3,3,0);
    cVecToso3(vec, theta, wx);
    matrix_set_block(se3mat, 0, 0, wx);

    // 速度分量
    matrix_get_block(V, 3, 0, vec);
	matrix_scale(vec, theta);
	matrix_set_block(se3mat, 0, 3, vec);

    matrix_delete(vec);
    matrix_delete(wx);
    return 0;
}

// Returns the 6-vector twist corresponding to an se(3) matrix se3mat.
int cSe3ToVec(const MatrixXd *se3mat, MatrixXd *V) {
    MatrixXd *vec = matrix_new(3,1,0.0);

    // 旋转分量: so3->vec
    MatrixXd *wx = matrix_new(3,3,0.0);
    matrix_get_block(se3mat, 0, 0, wx);
	cSo3ToVec(wx, vec);
    matrix_set_block(V, 0, 0, vec);

    // 速度分量: 直接赋值
    matrix_get_block(se3mat, 0, 3, vec);
    matrix_set_block(V, 3, 0, vec);
    
    matrix_delete(vec);
    matrix_delete(wx);

    return 0;
}

// Computes the 6x6 adjoint representation [AdT] of the homogeneous transformation matrix T.
int cAdjoint(const MatrixXd *T, MatrixXd *AdT) {
    MatrixXd *R = matrix_new(3,3,0.0);
    MatrixXd *p = matrix_new(3,1,0.0);
    cTransToRp(T, R, p);

    matrix_set_block(AdT, 0, 0, R);
    matrix_set_block(AdT, 3, 3, R);

    MatrixXd *px = matrix_new(3,1,0.0);
    cVecTose3(p, 1.0, px);
    MatrixXd *pxR = matrix_new(3,3,0.0);
    matrix_multiply(px, R, pxR);
    matrix_set_block(AdT, 3, 0, pxR);

    matrix_set_all(R, 0.0);
    matrix_set_block(AdT, 0, 3, R);

    matrix_delete(R);
    matrix_delete(p);
    matrix_delete(px);
    matrix_delete(pxR);

    return 0;
}

// Returns a normalized screw axis representation S of a screw described by 
// a unit vector s in the direction of the screw axis, located at the point q, with pitch h.
int cScrewToAxis(const MatrixXd *q, const MatrixXd *s, double h, int infiniteH, MatrixXd *screw) {
    matrix_set_all(screw, 0.0);

    MatrixXd *omg = matrix_new_copy(s);
    int ret = matrix_normalize(omg);

    // 平动 + 转动
    if (infiniteH <= 0) {
        MatrixXd *vel = matrix_new(3,1,0.0);
        matrix_set_block(screw, 0,0, omg);
        
        // w x q
        MatrixXd *v = matrix_new(3,1,0.0);
        matrix_multiply(omg, q, v);
        // hw
        vel = matrix_new_copy(omg);
        matrix_scale(vel, h);
        // vel = -w x q + hw
        matrix_plus(-1.0, v, 1.0, vel, vel);

        matrix_set_block(screw, 3, 0, vel);
        matrix_delete(vel);
        matrix_delete(v);
    }
    // 纯平动
    else {
        matrix_set_block(screw, 3, 0, omg);
    }

    return 0;
}

// Extracts the normalized screw axis S and the distance traveled along the screw q
// from the 6-vector of exponential coordinates Sq.
double cAxisAng6(const MatrixXd *expc6, MatrixXd *S) {
    matrix_set_all(S, 0.0);

    MatrixXd *omg = matrix_new(3,1,0.0);
    matrix_get_block(expc6, 0, 0, omg);
    MatrixXd *vel = matrix_new(3,1,0.0);
    matrix_get_block(expc6, 3, 0, vel);

    double theta = 0.0;
    // 平动分量
    double dis = matrix_inner_product(omg, vel);
    double qW = matrix_norm(omg);
    double qV = matrix_norm(vel);

    // 无运动
    if (qW < auxEPS && fabs(dis) < auxEPS) {
        matrix_set(S, 2, 0, 1.0);
    }
    // 有转动
    else if (qW > auxEPS) {
        theta = qW;
        matrix_copy(expc6, S);
        matrix_scale(S, 1.0 / qW);
    }
    // 纯平动
    else {
        theta = qV;
        matrix_normalize(vel);
        matrix_set_block(S, 3, 0, vel);
    }

    matrix_delete(omg);
    matrix_delete(vel);

    return theta;
}

// Computes the homogeneous transformation matrix T SE(3) corresponding to 
// the matrix exponential of se3mat se(3).
int cMatrixExp6(const MatrixXd *se3mat, MatrixXd *T) {
    matrix_set_identity(T);

    MatrixXd *V = matrix_new(6,1, 0.0);
    cSe3ToVec(se3mat, V);
    MatrixXd *omg = matrix_new(3,1,0.0);
    matrix_get_block(V, 0, 0, omg);
    double theta = matrix_norm(omg);

	if (theta > auxEPS) {
		// so3
		MatrixXd *wx = matrix_new(3, 3, 0.0);
		cVecToso3(omg, 1.0, wx);
		// SO3
		MatrixXd *R = matrix_new(3, 3, 0.0);
		cMatrixExp3(wx, R);
		matrix_set_block(T, 0, 0, R);

		// 单位化后计算速度分量
		matrix_normalize(omg);
		matrix_scale(wx, 1.0 / theta);

		// (q-sq)w^2
		matrix_multiply(wx, wx, R);
		matrix_scale(R, theta - sin(theta));
		// (1-sq)w
		matrix_plus(1.0 - cos(theta), wx, 1.0, R, R);
		// Iq
		matrix_set_identity(wx);
		matrix_plus(theta, wx, 1.0, R, R);

		MatrixXd *vel = matrix_new(3, 1, 0.0);
		matrix_get_block(V, 3, 0, vel);
		matrix_multiply(R, vel, omg);
		matrix_set_block(T, 0, 3, omg);

		matrix_delete(wx);
		matrix_delete(R);
		matrix_delete(vel);
	}
	else {
		MatrixXd *R = matrix_new(3, 3, 0.0);
		matrix_set_block(T, 0, 0, R);
		matrix_delete(R);
	}

    matrix_delete(V);
    matrix_delete(omg);

    return 0;
}


// Computes the matrix logarithm se3mat se(3) of the homogeneous transformation matrix T SE(3).
int cMatrixLog6(const MatrixXd *T, MatrixXd *se3mat) {
    MatrixXd *R = matrix_new(3,3,0.0);
    MatrixXd *p = matrix_new(3,1,0.0);
    cTransToRp(T, R, p);
    MatrixXd *V = matrix_new(6,1, 0.0);

    MatrixXd *so3 = matrix_new(3,3,0.0);
    cMatrixLog3(R, so3);
    MatrixXd *omg = matrix_new(3,1, 0.0);
    cSo3ToVec(so3, omg);
    double sum = 0.0;
    for (int i=0; i<3; ++i) {
        sum += fabs(omg->data[i]);
    }
    double dis = matrix_norm(p);

    // 有转动
    if (sum > auxEPS) {
		matrix_set_block(V, 0, 0, omg);

        double theta = matrix_norm(omg);
        matrix_normalize(omg);
        MatrixXd *wx = matrix_new(3,3,0.0);
        cVecToso3(omg, 1.0, wx);
        matrix_multiply(wx, wx, so3);
        matrix_plus(-1/2.0, wx, 1.0/theta - 1.0/tan(theta/2.0)/2.0, so3, R);
        matrix_set_identity(so3);
        matrix_plus(1.0/theta, so3, 1.0, R, R);

        matrix_multiply(R, p, omg);
		matrix_scale(omg, theta);
        matrix_set_block(V, 3, 0, omg);

        matrix_delete(wx);
    }
    // 无转动
    else if (dis > auxEPS) {
        matrix_normalize(p);
        matrix_set_block(V, 3, 0, p);
    }
    
    cVecTose3(V, 1.0, se3mat);

    matrix_delete(V);
    matrix_delete(R);
    matrix_delete(p);
    matrix_delete(so3);
    matrix_delete(omg);

    return 0;
}

