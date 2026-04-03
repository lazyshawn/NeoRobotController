
#include "AuxTransformation.h"

#include <gtest/gtest.h>

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
