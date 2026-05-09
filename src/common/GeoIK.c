
#include "GeoIK.h"

#include "AuxMatrix.h"
#include "AuxRotRep.h"
#include "GeoIK_subproblem.h"

static const double dim_ESP = 1e-9;

// 消去 TCP 和基座偏移，计算 R06 和 p06
int get_transform06(const ArmKineConfig cfg, const double T[4][4], MatrixXd *R06, MatrixXd *p06) {
	MatrixXd *R6T = matrix_new(3, 3, 0.0);
	MatrixXd *p6T = matrix_new(3, 1, 0.0);
	MatrixXd *p01 = matrix_new(3, 1, 0.0);

	for (int i = 0; i < 3; ++i) {
		// R0T
		for (int j = 0; j < 3; ++j) {
			matrix_set(R06, i, j, T[i][j]);
		}
		// p0T
		matrix_set(p06, i, 0, T[i][3]);
		// p6T
		matrix_set(p6T, i, 0, cfg.tcp[i]);
		// p01
		matrix_set(p01, i, 0, cfg.jntAxisOffset[0][i]);
	}
	// R6T
	double euler[3];
	for (int i = 0; i < 3; ++i) {
		euler[i] = cfg.tcp[3 + i];
	}
	Euler2Rot(euler, R6T->data);
	matrix_transpose(R6T);

	// R06 = R0T RT6
	matrix_multiply(R06, R6T, R06);

	// p06 = p0T - p01 - R06 p6T
	matrix_plus(1.0, p06, -1.0, p01, p06);
	matrix_multiply(R06, p6T, p01);
	matrix_plus(1.0, p06, -1.0, p01, p06);

	matrix_delete(R6T);
	matrix_delete(p6T);
	matrix_delete(p01);
	return 0;
}

int GeoIK_3intersect_with_2parallel(const ArmKineConfig cfg, const double T[4][4], double theta[][6]) {
	int num = 0;

	MatrixXd *R06 = matrix_new_identity(3);
	MatrixXd *p06 = matrix_new(3, 1, 0.0);
	MatrixXd *p12 = matrix_from_array(3, 1, cfg.jntAxisOffset[1], 3);
	MatrixXd *p23 = matrix_from_array(3, 1, cfg.jntAxisOffset[2], 3);
	MatrixXd *p34 = matrix_from_array(3, 1, cfg.jntAxisOffset[3], 3);

	MatrixXd *h1 = matrix_from_array(3, 1, cfg.jntDir[0], 3);
	MatrixXd *h2 = matrix_from_array(3, 1, cfg.jntDir[1], 3);
	MatrixXd *h3 = matrix_from_array(3, 1, cfg.jntDir[2], 3);
	MatrixXd *h4 = matrix_from_array(3, 1, cfg.jntDir[3], 3);
	MatrixXd *h5 = matrix_from_array(3, 1, cfg.jntDir[4], 3);

	MatrixXd *h6 = matrix_from_array(3, 1, cfg.jntDir[5], 3);
	MatrixXd *pos = matrix_new(3, 1, 0.0);
	MatrixXd *Rij = matrix_new(3, 3, 0.0);
	MatrixXd *Rs = matrix_new(3, 3, 0.0);
	MatrixXd *R56 = matrix_new(3, 3, 0.0);

	MatrixXd *so3 = matrix_new(3, 3, 0.0);
	MatrixXd *p0 = matrix_new(3, 1, 0.0);
	MatrixXd *p1 = matrix_new(3, 1, 0.0);

	get_transform06(cfg, T, R06, p06);

	// Step 1. solve q1 using sub4
	double q1[2] = { 0.0, 0.0 };
	int num0 = 0;
	// p0 = p12 + p23 + p34
	matrix_plus(1.0, p12, 1.0, p23, p0);
	matrix_plus(1.0, p0, 1.0, p34, p0);
	double proj = matrix_inner_product(p0, h2);
	canonical_subproblem_4(p06->data, h1->data, h2->data, proj, q1, &num0);

	// Step 2. for each q1, use sub3 to find up to 2 solutions of q3
	double q3[2] = { 0.0 };
	int num3 = 0;
	// p0 = -p23
	matrix_copy(p23, p0);
	matrix_scale(p0, -1.0);
	for (int i = 0; i < num0; ++i) {
		// dist = || R10 p06 - p12||
		matrix_rodrigues(h1, p06, q1[i], p1);
		matrix_plus(1.0, p1, -1.0, p12, p1);
		proj = matrix_norm(p1);

		canonical_subproblem_3(p34->data, h3->data, p0->data, proj, q3, &num3);

		// 保存 (q1, q3)
		for (int j = 0; j < num3; ++j) {
			theta[num][0] = - q1[i];
			theta[num][2] = q3[j];
			num++;
		}
	}

    // Step 3. for each (q1, q3), use sub1 to solve q2
	double q2 = 0.0;
	int num2 = 0;
	for (int i = 0; i < num; ++i) {
		// p1 = R23 p34 + p23
		matrix_rodrigues(h3, p34, theta[i][2], p1);
		matrix_plus(1.0, p1, 1.0, p23, p1);
		// p0 = R10 p06 - p12
		matrix_rodrigues(h1, p06, -theta[i][0], p0);
		matrix_plus(1.0, p0, -1.0, p12, p0);
		
		canonical_subproblem_1(p1->data, h2->data, p0->data, &q2, &num2);

		theta[i][1] = q2;
	}

	// Step 4. solve for up to 2 solutions of (q4, q5) using sub2
	double q4[2] = { 0.0 }, q5[2] = { 0.0 };
	int num45 = 0, num123 = num;
	for (int i = 0; i < num123; ++i) {
		// Rs = R36 = R32 R21 R10 R06
		matrix_axis_angle(h1, -theta[i][0], Rij);
		matrix_multiply(Rij, R06, Rs);
		matrix_axis_angle(h2, -theta[i][1], Rij);
		matrix_multiply(Rij, Rs, Rs);
		matrix_axis_angle(h3, -theta[i][2], Rij);
		matrix_multiply(Rij, Rs, Rs);

		// p1 = R32 R21 R10 R06 h6 = Rs h6; p0 = h6
		matrix_multiply(Rs, h6, p1);
		canonical_subproblem_2(p1->data, h4->data, h6->data, h5->data, q4, q5, &num45);
		theta[i][3] = -q4[0];
		theta[i][4] = q5[0];

		// R56 = R54 R43 Rs
		matrix_axis_angle(h4, -theta[i][3], Rij);
		matrix_multiply(Rij, Rs, R56);
		matrix_axis_angle(h5, -theta[i][4], Rij);
		matrix_multiply(Rij, R56, R56);
		// Step 5. Solve q6, 找任意不在h6上的点，计算 R56 h6，再使用sub1求解 q6.
		double q6 = 0.0;
		int num6 = 0;
		matrix_plus(1.0, h5, 1.0, h6, p1);
		matrix_multiply(R56, p1, p0);
		canonical_subproblem_1(p1->data, h6->data, p0->data, &q6, &num6);
		theta[i][5] = q6;

		for (int j = 1; j < num45; ++j) {
			theta[num][0] = theta[i][0];
			theta[num][1] = theta[i][1];
			theta[num][2] = theta[i][2];
			theta[num][3] = - q4[j];
			theta[num][4] = q5[j];

			// R56 = R54 R43 Rs
			matrix_axis_angle(h4, -theta[num][3], Rij);
			matrix_multiply(Rij, Rs, R56);
			matrix_axis_angle(h5, -theta[num][4], Rij);
			matrix_multiply(Rij, R56, R56);
			// Step 5. Solve q6
			matrix_plus(1.0, h5, 1.0, h6, p1);
			matrix_multiply(R56, p1, p0);
			canonical_subproblem_1(p1->data, h6->data, p0->data, &q6, &num6);
			theta[num][5] = q6;

			num++;
		}
	}

	matrix_delete(R06);
	matrix_delete(p06);
	matrix_delete(pos);
    matrix_delete(Rij);
    matrix_delete(Rs);

    matrix_delete(R56);
    matrix_delete(so3);
    matrix_delete(p0);
    matrix_delete(p1);
    matrix_delete(h1);

    matrix_delete(h2);
    matrix_delete(h3);
    matrix_delete(h4);
    matrix_delete(h5);
    matrix_delete(h6);

    matrix_delete(p12);
    matrix_delete(p23);
    matrix_delete(p34);

	return num;
}

int GeoIK_3parallel_with_2intersect(const ArmKineConfig cfg, const double T[4][4], double theta[][6]) {
	int num = 0;

	MatrixXd *R06 = matrix_new_identity(3);
	MatrixXd *p06 = matrix_new(3, 1, 0.0);

	MatrixXd *p1 = matrix_new(3, 1, 0.0);
	MatrixXd *p2 = matrix_new(3, 1, 0.0);
	MatrixXd *k1 = matrix_new(3, 1, 0.0);
	MatrixXd *k2 = matrix_new(3, 1, 0.0);

	MatrixXd *pos = matrix_new(3, 1, 0.0);

	get_transform06(cfg, T, R06, p06);

	// Step 1. solve q1 using sub4
	double q1[2] = { 0.0 };
	int num1 = 0;
	matrix_set_all(p2, 0.0);
	// d = h2T (p12 + p23 + p34 + p45)
	for (int i = 1; i < 5; ++i) {
		matrix_copy_array(pos, cfg.jntAxisOffset[i], 3);
		matrix_plus(1.0, p2, 1.0, pos, p2);
	}
	matrix_copy_array(pos, cfg.jntDir[5], 3);
	double proj = matrix_inner_product(p2, pos);
	// h2T R10 p06 = d
	matrix_copy_array(k1, cfg.jntDir[0], 3);
	matrix_copy_array(k2, cfg.jntDir[1], 3);
	canonical_subproblem_4(p06->data, k1->data, k2->data, proj, q1, &num1);

	matrix_delete(R06);
	matrix_delete(p06);
	matrix_delete(p1);
	matrix_delete(p2);
	matrix_delete(k1);
	matrix_delete(k2);
	matrix_delete(pos);
	return num;
}
