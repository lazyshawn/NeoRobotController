
#include "GeoIK_subproblem.h"
#include "AuxMatrix.h"

static const double dim_EPS = 1e-6;

int canonical_subproblem_1(
    const double *pPos,
    const double *dir,
    const double *qPos,
    double *theta,
    int *num
) {
    int ans = 0;
    MatrixXd *k = matrix_from_array(3, 1, dir, 3);
    MatrixXd *p = matrix_from_array(3, 1, pPos, 3);
    MatrixXd *q = matrix_from_array(3, 1, qPos, 3);
    MatrixXd *x = matrix_new(3, 1, 0.0);
    MatrixXd *y = matrix_new(3, 1, 0.0);
	
    // k 和 p 不共线
	matrix_outer_product(k, p, x);
    if (matrix_norm(x) < dim_EPS)
        return -1;
    // k 和 q 不共线
	matrix_outer_product(k, q, x);
	if (matrix_norm(x) < dim_EPS)
        return -2;
    
    // y轴: kxp
    matrix_outer_product(k, p, y);
    double yproj = matrix_inner_product(y, q);
    // x轴: -kxkxp = -kxy = yxk
    matrix_outer_product(y, k, x);
    double xproj = matrix_inner_product(x, q);
    // 最近解
    theta[0] = atan2(yproj, xproj);

    matrix_delete(k);
    matrix_delete(p);
    matrix_delete(q);
    matrix_delete(x);
    matrix_delete(y);
    return 0;
}

int canonical_subproblem_2(
    const double *pos1,
    const double *dir1,
    const double *pos2,
    const double *dir2,
    double *theta1,
    double *theta2,
	int *num
) {
    MatrixXd *k1 = matrix_from_array(3,1,dir1,3);
    MatrixXd *k2 = matrix_from_array(3,1,dir2,3);
    MatrixXd *p1 = matrix_from_array(3,1,pos1,3);
    MatrixXd *p2 = matrix_from_array(3,1,pos2,3);

    // (k1,p1), (k2,p2), (k1,k2) 三组向量均不共线

	// 解q1
	int num1 = 0;
	double z = matrix_inner_product(k2, p2);
	canonical_subproblem_4(p1->data, k1->data, k2->data, z, theta1, &num1);

	// 解q2
	int num2 = 0;
	z = matrix_inner_product(k1, p1);
	canonical_subproblem_4(p2->data, k2->data, k1->data, z, theta2, &num2);

	// q1q2 分别有 1 组和 2 组解
	if (num1 != num2) {
		*num = 2;
		if (num1 == 1)
			theta1[1] = theta1[0];
		else if (num2 == 1)
			theta2[1] = theta2[0];
	}
	else if (num1 == 1) {
		*num = 1;
	}
	// 都有两组解，匹配更近的解为一组
	else if (num1 == 2) {
		*num = 2;
		MatrixXd *p1a = matrix_new(3, 1, 0.0);
		MatrixXd *p1b = matrix_new(3, 1, 0.0);
		MatrixXd *p2a = matrix_new(3, 1, 0.0);

		matrix_rodrigues(k1, p1, theta1[0], p1a);
		matrix_rodrigues(k1, p1, theta1[1], p1b);
		matrix_rodrigues(k2, p2, theta2[0], p2a);

		matrix_plus(1.0, p1a, -1.0, p2a, k1);
		matrix_plus(1.0, p1b, -1.0, p2a, k2);
		double norm1 = matrix_squared_norm(k1);
		double norm2 = matrix_squared_norm(k2);

		// p1b 离 p2a 更近，交换 p1 中两个解的位置
		if (norm2 < norm1) {
			double tmp = theta1[0];
			theta1[0] = theta1[1];
			theta1[1] = tmp;
		}

		matrix_delete(p1a);
		matrix_delete(p1b);
		matrix_delete(p2a);
	}

    matrix_delete(k1);
    matrix_delete(k2);
    matrix_delete(p1);
    matrix_delete(p2);
    return 1;
}

int canonical_subproblem_3(
	const double *pos1,
	const double *dir1,
	const double *pos2,
	double dist,
	double *theta,
	int *num
) {
	int ans = 0;
    MatrixXd *k = matrix_from_array(3,1,dir1,3);
    MatrixXd *p1 = matrix_from_array(3,1,pos1,3);
    MatrixXd *p2 = matrix_from_array(3,1,pos2,3);

	// (k, p1), (k, p2) 不共线

    // 转化为子问题 4
    dist = (matrix_squared_norm(p1) + matrix_squared_norm(p2) - dist*dist) / 2.0;
    ans = canonical_subproblem_4(pos1, dir1, pos2, dist, theta, num);

    matrix_delete(k);
    matrix_delete(p1);
    matrix_delete(p2);
	return ans;
}

int canonical_subproblem_4(
    const double *pos,
    const double *kDir,
    const double *hDir,
    double dist,
    double *theta,
    int *num
) {
    int ans = 0;
    MatrixXd *p = matrix_from_array(3,1,pos,3);
    MatrixXd *k = matrix_from_array(3,1,kDir,3);
    MatrixXd *h = matrix_from_array(3,1,hDir,3);
    MatrixXd *ht = matrix_from_array(1,3,hDir,3);
    MatrixXd *x = matrix_new(3,1,0.0);
    MatrixXd *y = matrix_new(3,1,0.0);
    MatrixXd *Akp = matrix_new(3,2,0.0);
    MatrixXd *A = matrix_new(1,2,0.0);
    MatrixXd *xmin = matrix_new(2,1,0.0);
    MatrixXd *xn = matrix_new(2,1,0.0);

	// (p,k), (h,k) 不共线，否则实际投影距离与theta无关，有无数近似解

    // Akp = [kxp, yxk]
    matrix_outer_product(k, p, y);
    matrix_set_block(Akp, 0,0,y);
    matrix_outer_product(y, k, x);
    matrix_set_block(Akp, 0,1,x);
    // A = ht_Akp
    matrix_multiply(ht, Akp, A);
    // b
    double b = dist - matrix_inner_product(h, k) * matrix_inner_product(k, p);
    // xmin = At / ||At||2 * b
    double a0 = matrix_at(A, 0, 0), a1 = matrix_at(A, 0, 1);
	double nA2 = a0 * a0 + a1 * a1;
    matrix_set(xmin, 0, 0, a0 * b / nA2);
    matrix_set(xmin, 1, 0, a1 * b / nA2);
    // xn = [-a1; a0]
    matrix_set(xn, 0, 0, -a1);
    matrix_set(xn, 1, 0, a0);

    // 解的模长: xmin = n[sq; cq]
    double normXmin = matrix_norm(xmin);
	// ||xmin|| < 1: 两组精确解
	if (1 - normXmin > dim_EPS) {
		*num = 2;
		double det = sqrt(nA2 - b * b) / nA2;
		double sq = matrix_at(xmin, 0, 0) + det * matrix_at(xn, 0, 0);
		double cq = matrix_at(xmin, 1, 0) + det * matrix_at(xn, 1, 0);
		theta[0] = atan2(sq, cq);
		sq = matrix_at(xmin, 0, 0) - det * matrix_at(xn, 0, 0);
		cq = matrix_at(xmin, 1, 0) - det * matrix_at(xn, 1, 0);
		theta[1] = atan2(sq, cq);
	}
	// ||xmin|| == 1: 一组精确解, ||xmin|| > 1: 一组近似解
	else {
        if (normXmin - 1 > dim_EPS) {
            ans = 1;
        }
		*num = 1;
		theta[0] = atan2(matrix_at(xmin, 0, 0), matrix_at(xmin, 1, 0));
	}

    matrix_delete(xmin);
    matrix_delete(xn);
    matrix_delete(x);
    matrix_delete(y);
    matrix_delete(Akp);
    matrix_delete(A);
    matrix_delete(ht);
    matrix_delete(k);
    matrix_delete(p);
    matrix_delete(h);

    return ans;
}


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
) {
    int ans = 0;

    MatrixXd *p0 = matrix_from_array(3,1,shift1,3);
    MatrixXd *p1 = matrix_from_array(3,1,pos1,3);
    MatrixXd *p2 = matrix_from_array(3,1,shift3,3);
    MatrixXd *p3 = matrix_from_array(3,1,pos3,3);
    MatrixXd *k1 = matrix_from_array(3,1,dir1,3);
    MatrixXd *k2 = matrix_from_array(3,1,dir2,3);
    MatrixXd *k3 = matrix_from_array(3,1,dir3,3);
    MatrixXd *ps = matrix_new(3,1,0.0);

    // ||k1xp1||2
    double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, a4 = 0.0;
    matrix_outer_product(k1, p1, ps);
    a0 += matrix_squared_norm(ps);
    // ||p1s||2
    matrix_plus(1.0, p0, matrix_inner_product(k1, p1), k1, ps);
    a0 += matrix_squared_norm(ps);







    matrix_delete(p0);
    matrix_delete(p1);
    matrix_delete(p2);
    matrix_delete(p3);
    matrix_delete(k1);
    matrix_delete(k2);
    matrix_delete(k3);

    return ans;
}
