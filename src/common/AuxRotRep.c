#include "AuxRotRep.h"

// M_PI
#include "AuxMatrix.h"

static const double dim_EPS = 1e-6;

// 欧拉角转旋转矩阵
int Euler2Rot(const double euler[3], double R[9]) {
    double a = euler[0], b = euler[1], c = euler[2];
    double sa = sin(a), sb = sin(b), sc = sin(c);
    double ca = cos(a), cb = cos(b), cc = cos(c);
    
    R[0] = ca*cb;
    R[1] = ca*sb*sc - sa*cc;
    R[2] = ca*sb*cc + sa*sc;

    R[3] = sa*cb;
    R[4] = sa*sb*sc + ca*cc;
    R[5] = sa*sb*cc - ca*sc;

    R[6] = -sb;
    R[7] = cb*sc;
    R[8] = cb*cc;

    return 0;
}

// 旋转矩阵转欧拉角
int Rot2Euler(const double R[9], double euler[3]) {
    // 规避欧拉角死锁问题: b = pi/2, a+c = (+/-)atan(r12, r22)
	if (R[6] < -1.0 + dim_EPS) {
		euler[0] = 0.0;
		euler[1] = M_PI / 2 - 1e-3;
		euler[2] = atan2(R[1], R[4]);
		return 1;
	}
	else if (R[6] > 1.0 - dim_EPS) {
		euler[0] = 0.0;
		euler[1] = -M_PI / 2 + 1e-3;
		euler[2] = -atan2(R[1], R[4]);
		return 1;
	}

    // atan2(-r31, sqrt(r11^2 + r21^2))
    euler[1] = atan2(-R[6], sqrt(R[0]*R[0] + R[3]*R[3]));
    euler[0] = atan2(R[3], R[0]);
    euler[2] = atan2(R[7], R[8]);
    
    return 0;
}

// 四元数转旋转矩阵
int Quat2Rot(const double q[4], double R[9]) {
	double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

	R[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	R[1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	R[2] = 2 * (q[0] * q[2] + q[1] * q[3]);

	R[3] = 2 * (q[0] * q[3] + q[1] * q[2]);
	R[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	R[5] = 2 * (q[2] * q[3] - q[0] * q[1]);

	R[6] = 2 * (q[1] * q[3] - q[0] * q[2]);
	R[7] = 2 * (q[0] * q[1] + q[2] * q[3]);
	R[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	for (int i = 0; i < 9; ++i) {
		R[i] /= n2;
	}

	return 0;
}

// 旋转矩阵转四元数
int Rot2Quat(const double R[9], double q[4]) {
	q[0] = sqrt(1.0 + R[0] + R[4] + R[8]) / 2;
	q[1] = (R[7] - R[5]) / (4 * q[0]);
	q[2] = (R[2] - R[6]) / (4 * q[0]);
	q[3] = (R[3] - R[1]) / (4 * q[0]);
	return 0;
}

// 四元数球面距离
double QuatDistance(const double q0[4], const double q1[4]) {
    double dot = 0.0;
    for (int i=0; i<4; ++i) {
        dot += q0[i] * q1[i];
    }
    return 2 * acos(fabs(dot));
}

// 四元数球面插补
int QuatSlerp(const double q0[4], const double q1[4], double t, double q[4]) {
    double dist = QuatDistance(q0, q1);
    double dot = 0.0;
        for (int i=0; i<4; ++i) {
        dot += q0[i] * q1[i];
    }
    int sign = dot < 0.0 ? -1 : 1;
    // 反转 q1 方向，避免绕远
    double qs[4];
    for (int i=0; i<4; ++i) {
        qs[i] = sign * q1[i];
    }

    // 两个四元数几乎相同，线性插值并归一化
    if (dist < dim_EPS) {
        double norm = 0.0;
        for (int i=0; i<4; ++i) {
            q[i] = q0[i] + t * (qs[i] - q0[i]);
            norm += q[i] * q[i];
        }
        norm = sqrt(norm);
        for (int i=0; i<4; ++i) {
            q[i] /= norm;
        }
        return 1;
    }

    double k1 = sin((1.0-t) * dist) / sin(dist);
    double k2 = sin(t * dist) / sin(dist);
    for (int i=0; i<4; ++i) {
        q[i] = k1 * q0[i] + k2 * qs[i];
    }

    return 0;
}
