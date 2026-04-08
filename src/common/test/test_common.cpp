
#include "AuxTransformation.h"
#include "AuxKinematics.h"

#include <gtest/gtest.h>

const double nearThread = 1e-12;

// Demonstrate some basic assertions.
//TEST(CommonTest, BasicAssertions) {
//	// Expect two strings not to be equal.
//	EXPECT_STRNE("hello", "world");
//	// Expect equality.
//	EXPECT_EQ(7 * 6, 42);
//}


TEST(CommonTest, Transformation) {
	MatrixXd *Tsb = matrix_new_identity(4);
	MatrixXd *Tsc = matrix_new_identity(4);

	matrix_set(Tsb, 0, 0, cos(M_PI / 6));
	matrix_set(Tsb, 0, 1, -sin(M_PI / 6));
	matrix_set(Tsb, 1, 0, sin(M_PI / 6));
	matrix_set(Tsb, 1, 1, cos(M_PI / 6));
	matrix_set(Tsb, 0, 3, 1);
	matrix_set(Tsb, 1, 3, 2);
	//matrix_cout(Tsb);

	matrix_set(Tsc, 0, 0, cos(M_PI / 3));
	matrix_set(Tsc, 0, 1, -sin(M_PI / 3));
	matrix_set(Tsc, 1, 0, sin(M_PI / 3));
	matrix_set(Tsc, 1, 1, cos(M_PI / 3));
	matrix_set(Tsc, 0, 3, 2);
	matrix_set(Tsc, 1, 3, 1);
	//matrix_cout(Tsc);

	MatrixXd *T = matrix_new_identity(4);
	cTransInv(Tsb, T);
	//matrix_cout(T);

	matrix_multiply(Tsc, T, Tsb);
	//matrix_cout(Tsb);

	MatrixXd *se3 = matrix_new(4, 4, 0);
	cMatrixLog6(Tsb, se3);
	//matrix_cout(se3);

	MatrixXd *V = matrix_new(6, 1, 0), *S = matrix_new(6, 1, 0);
	cSe3ToVec(se3, V);
	//matrix_cout(V);

	double theta = cAxisAng6(V, S);
	//matrix_cout(S);
	//std::cout << "theta = " << theta << std::endl;

	EXPECT_EQ(theta, M_PI/6);
	EXPECT_EQ(matrix_at(S, 0, 0), 0);
	EXPECT_EQ(matrix_at(S, 1, 0), 0);
	EXPECT_EQ(matrix_at(S, 2, 0), 1);
	EXPECT_EQ(matrix_at(S, 3, 0), 3.3660254037844384);
	EXPECT_EQ(matrix_at(S, 4, 0), -3.3660254037844393);
	EXPECT_EQ(matrix_at(S, 5, 0), 0);
}

TEST(CommonTest, FKine) {
	// Link: 430, 163.5984, 821.7770, 210.8518, 1029.1985, 115
	// TCP:  88.6768, 728.5914, -2.5914

	// --- 1. FKinSpace
	double M0[4][4] = { 0.0 };
	M0[0][0] = 1; M0[0][1] = 0; M0[0][2] = 0; M0[0][3] = 1192.7969 + 115 + 728.5914;
	M0[1][0] = 0; M0[1][1] = 1; M0[1][2] = 0; M0[1][3] = 0 - 2.5914;
	M0[2][0] = 0; M0[2][1] = 0; M0[2][2] = 1; M0[2][3] = 1032.6288 - 88.6768;
	M0[3][0] = 0; M0[3][1] = 0; M0[3][2] = 0; M0[3][3] = 1;

	double Slist[6][6] = { 0.0 };
	Slist[0][0] = 0; Slist[0][1] = 0; Slist[0][2] = 1; Slist[0][3] = 0; Slist[0][4] = 0; Slist[0][5] = 0;
	Slist[1][0] = 0; Slist[1][1] = 1; Slist[1][2] = 0; Slist[1][3] = 0; Slist[1][4] = 0; Slist[1][5] = 163.5984;
	Slist[2][0] = 0; Slist[2][1] = 1; Slist[2][2] = 0; Slist[2][3] = -821.777; Slist[2][4] = 0; Slist[2][5] = 163.5984;
	Slist[3][0] = 1; Slist[3][1] = 0; Slist[3][2] = 0; Slist[3][3] = 0; Slist[3][4] = 1032.6288; Slist[3][5] = 0;
	Slist[4][0] = 0; Slist[4][1] = 1; Slist[4][2] = 0; Slist[4][3] = -1032.6288; Slist[4][4] = 0; Slist[4][5] = 1192.7969;
	Slist[5][0] = 1; Slist[5][1] = 0; Slist[5][2] = 0; Slist[5][3] = 0; Slist[5][4] = 1032.6288; Slist[5][5] = 0;

	double thetalist[6] = { 10, -20, 30, -40, 50, -60 };
	for (int i = 0; i < 6; ++i) {
		thetalist[i] *= M_PI / 180;
	}

	double T[4][4] = { 0.0 };
	FKinSpace(M0, Slist, thetalist, 6, T);

	// 位置校验
	EXPECT_NEAR(T[0][3], 1419.0886056004786, nearThread);
	EXPECT_NEAR(T[1][3], -249.97753741763282, nearThread);
	EXPECT_NEAR(T[2][3], 254.08912457483615, nearThread);


	// --- 2. Slist to Blist
	MatrixXd *M0mat = matrix_new(4, 4, 0.0);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_set(M0mat, i, j, M0[i][j]);
		}
	}
	MatrixXd *Mbs = matrix_new(4, 4, 0.0);
	cTransInv(M0mat, Mbs);
	MatrixXd *AdT = matrix_new(6, 6, 0.0);
	cAdjoint(Mbs, AdT);
	//matrix_cout(AdT);

	double Blist[6][6] = { 0.0 };
	for (int i = 0; i < 6; ++i) {
		MatrixXd *sVec = matrix_from_array(6, 1, Slist[i], 6);
		MatrixXd *bVec = matrix_new(6, 1, 0);
		matrix_multiply(AdT, sVec, bVec);
		matrix_to_array(bVec, Blist[i], 6);
		matrix_delete(sVec);
		matrix_delete(bVec);
	}

	matrix_delete(AdT);
	matrix_delete(M0mat);
	matrix_delete(Mbs);


	// --- 3. FKinBody
	FKinBody(M0, Blist, thetalist, 6, T);

	// 位置校验
	EXPECT_NEAR(T[0][3], 1419.0886056004786, nearThread);
	EXPECT_NEAR(T[1][3], -249.97753741763282, nearThread);
	EXPECT_NEAR(T[2][3], 254.08912457483615, nearThread);
}

TEST(CommonTest, Jacobian) {
	// Morden robotics: Example 5.3
	double L1 = 100, L2 = 50;
	double thetalist[6] = { 10,-20,100,-30,40,-50 };
	for (int i = 0; i < 6; ++i) {
		thetalist[i] *= (i == 2) ? 1.0 : M_PI / 180;
	}

	// --- 理论值
	double s1 = sin(thetalist[0]), c1 = cos(thetalist[0]);
	double s2 = sin(thetalist[1]), c2 = cos(thetalist[1]);
	double s4 = sin(thetalist[3]), c4 = cos(thetalist[3]);
	double s5 = sin(thetalist[4]), c5 = cos(thetalist[4]);

	double Jans[6][6] = { 0.0 };
	Jans[0][2] = 1;
	Jans[1][0] = -c1; Jans[1][1] = -s1; Jans[1][3] = L1 * s1; Jans[1][4] = -L1 * c1;
	Jans[2][3] = -s1 * c2; Jans[2][4] = c1 * c2; Jans[2][5] = -s2;
	Jans[3][0] = -s1 * s2; Jans[3][1] = c1 * s2; Jans[3][2] = c2;
	Jans[4][0] = -c1 * c4 + s1 * c2*s4; Jans[4][1] = -s1 * c4 - c1 * c2*s4; Jans[4][2] = s2 * s4;
	Jans[5][0] = -c5 * (s1*c2*c4 + c1 * s4) + s1 * s2*s5; Jans[5][1] = c5 * (c1*c2*c4 - s1 * s4) - c1 * s2*s5; Jans[5][2] = -s2 * c4*c5 - c2 * s5;

	MatrixXd *ws = matrix_new(3, 1, 0.0);
	MatrixXd *vs = matrix_new(3, 1, 0.0);
	MatrixXd *qw = matrix_new(3, 1, 0.0);
	// 腕关节位置 qw
	matrix_set(qw, 0, 0, -(L2 + thetalist[2]) * s1*c2);
	matrix_set(qw, 1, 0, (L2 + thetalist[2]) * c1*c2);
	matrix_set(qw, 2, 0, L1 - (L2 + thetalist[2]) * s2);
	matrix_scale(qw, -1.0);
	// vs[3]
	matrix_copy_array(ws, Jans[3], 3);
	matrix_outer_product(ws, qw, vs);
	matrix_to_array(vs, &Jans[3][3], 3);
	// vs[4]
	matrix_copy_array(ws, Jans[4], 3);
	matrix_outer_product(ws, qw, vs);
	matrix_to_array(vs, &Jans[4][3], 3);
	// vs[5]
	matrix_copy_array(ws, Jans[5], 3);
	matrix_outer_product(ws, qw, vs);
	matrix_to_array(vs, &Jans[5][3], 3);
	matrix_delete(ws);
	matrix_delete(vs);
	matrix_delete(qw);

	// --- 1. Jacobian in Space
	double Slist[6][6];
	Slist[0][0] = 0; Slist[0][1] = 0; Slist[0][2] = 1; Slist[0][3] = 0; Slist[0][4] = 0; Slist[0][5] = 0;
	Slist[1][0] = -1; Slist[1][1] = 0; Slist[1][2] = 0; Slist[1][3] = 0; Slist[1][4] = -L1; Slist[1][5] = 0;
	Slist[2][0] = 0; Slist[2][1] = 0; Slist[2][2] = 0; Slist[2][3] = 0; Slist[2][4] = 1; Slist[2][5] = 0;
	Slist[3][0] = 0; Slist[3][1] = 0; Slist[3][2] = 1; Slist[3][3] = L2; Slist[3][4] = 0; Slist[3][5] = 0;
	Slist[4][0] = -1; Slist[4][1] = 0; Slist[4][2] = 0; Slist[4][3] = 0; Slist[4][4] = -L1; Slist[4][5] = L2;
	Slist[5][0] = 0; Slist[5][1] = 1; Slist[5][2] = 0; Slist[5][3] = -L1; Slist[5][4] = 0; Slist[5][5] = 0;

	double M0[4][4] = { 0.0 };
	M0[0][0] = 1; M0[0][3] = 0;
	M0[1][1] = 1; M0[1][3] = L2;
	M0[2][2] = 1; M0[2][3] = L1;
	M0[3][3] = 1;

	double J[6][6];
	JacobianSpace(Slist, thetalist, 6, J);
	// 校验
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			EXPECT_NEAR(J[i][j], Jans[i][j], nearThread);
		}
	}

	// --- 2. Slist to Blist
	MatrixXd *M0mat = matrix_new(4, 4, 0.0);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_set(M0mat, i, j, M0[i][j]);
		}
	}
	MatrixXd *Mbs = matrix_new(4, 4, 0.0);
	cTransInv(M0mat, Mbs);
	MatrixXd *AdT = matrix_new(6, 6, 0.0);
	cAdjoint(Mbs, AdT);
	//matrix_cout(AdT);

	double Blist[6][6] = { 0.0 };
	for (int i = 0; i < 6; ++i) {
		MatrixXd *sVec = matrix_from_array(6, 1, Slist[i], 6);
		MatrixXd *bVec = matrix_new(6, 1, 0);
		matrix_multiply(AdT, sVec, bVec);
		matrix_to_array(bVec, Blist[i], 6);
		matrix_delete(sVec);
		matrix_delete(bVec);
	}

	// FK
	double T[4][4];
	FKinSpace(M0, Slist, thetalist, 6, T);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_set(M0mat, i, j, T[i][j]);
		}
	}
	cTransInv(M0mat, Mbs);
	cAdjoint(Mbs, AdT);
	for (int i = 0; i < 6; ++i) {
		MatrixXd *sVec = matrix_from_array(6, 1, Jans[i], 6);
		MatrixXd *bVec = matrix_new(6, 1, 0);
		matrix_multiply(AdT, sVec, bVec);
		matrix_to_array(bVec, Jans[i], 6);
		matrix_delete(sVec);
		matrix_delete(bVec);
	}


	// --- Jacobian in Body
	JacobianBody(Blist, thetalist, 6, J);
	// 校验
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			EXPECT_NEAR(J[i][j], Jans[i][j], nearThread);
		}
	}

	matrix_delete(AdT);
	matrix_delete(M0mat);
	matrix_delete(Mbs);
}

TEST(CommonTest, IKine) {

}
