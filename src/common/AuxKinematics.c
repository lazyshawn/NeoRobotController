#include "AuxKinematics.h"
#include "AuxMatrix.h"
#include "AuxTransformation.h"

int FKinBody(
    const double M0[4][4],
    const double Blist[][6],
    const double *thetalist,
    int num,
    double T[4][4]
) {
	MatrixXd *Ti = matrix_new(4, 4, 0.0);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_set(Ti, i, j, M0[i][j]);
		}
	}
	MatrixXd *Tn = matrix_new(4, 4, 0.0);
	MatrixXd *se3mat = matrix_new(4, 4, 0.0);
	MatrixXd *tran = matrix_new(4, 4, 0.0);
	MatrixXd *vec = matrix_new(6, 1, 0.0);

    for (int i=0; i<num; ++i) {
        // Ti_i1
        matrix_copy_array(vec, Blist[i], 6);
        cVecTose3(vec, thetalist[i], se3mat);
        cMatrixExp6(se3mat, tran);
        // Ti
        matrix_multiply(Ti, tran, Tn);
        matrix_copy(Tn, Ti);
    }

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			T[i][j] = matrix_at(Tn, i, j);
		}
	}

	matrix_delete(Ti);
	matrix_delete(Tn);
	matrix_delete(se3mat);
	matrix_delete(tran);
	matrix_delete(vec);
    return 0;
}


int FKinSpace(
    const double M0[4][4],
    const double Slist[][6],
    const double *thetalist,
    int num,
    double T[4][4]
) {
    MatrixXd *Ti = matrix_new(4, 4, 0.0);
    for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) {
            matrix_set(Ti, i, j, M0[i][j]);
        }
    }
    MatrixXd *Tn = matrix_new(4, 4, 0.0);
    MatrixXd *se3mat = matrix_new(4,4,0.0);
    MatrixXd *tran = matrix_new(4, 4, 0.0);
    MatrixXd *vec = matrix_new(6,1,0.0);

    for (int i=num-1; i>=0; --i) {
        // Ti_i1
        matrix_copy_array(vec, Slist[i], 6);
        cVecTose3(vec, thetalist[i], se3mat);
        cMatrixExp6(se3mat, tran);
        // Ti
        matrix_multiply(tran, Ti, Tn);
        matrix_copy(Tn, Ti);
    }

    for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) {
            T[i][j] = matrix_at(Tn, i, j);
        }
    }

    matrix_delete(Ti);
    matrix_delete(Tn);
    matrix_delete(se3mat);
    matrix_delete(tran);
    matrix_delete(vec);

    return 0;
}
