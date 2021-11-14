/*
 * aux.c
 *
 *  Created on: 08 Nov 2021
 *      Author: martin
 */

#include "aux.h"

extern UART_HandleTypeDef huart2;

/* matrix manipulations */
void Mat_T(uint8_t rows, uint8_t cols, float result[cols][rows], float mat[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[j][i] = mat[i][j];
}

void Mat_Add(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[i][j] = mat1[i][j] + mat2[i][j];
}

void Mat_Sub(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[i][j] = mat1[i][j] - mat2[i][j];
}

void Mat_Mul(uint8_t rows, uint8_t cols, uint8_t add, float result[rows][cols], float mat1[rows][add], float mat2[add][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++) {
			result[i][j] = 0.0;
			for (uint8_t k = 0; k < add; k++)
				result[i][j] += mat1[i][k] * mat2[k][j];
		}
}

void Mat_Inv(uint8_t rows, float result[rows][rows], float mat[rows][rows]) {
	// calculate matrix of minors and determinant
	float minors[rows][rows];
	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < rows; j++) {
			float submat[rows-1][rows-1];
			uint8_t offsetK = 0;
			for (uint8_t k = 0; k < rows-1; k++) {
				uint8_t offsetL = 0;
				for (uint8_t l = 0; l < rows-1; l++) {
					if (k == i) offsetK = 1;
					if (l == j) offsetL = 1;
					submat[k][l] = mat[k+offsetK][l+offsetL];
				}
			}
			minors[i][j] = Mat_Det(rows-1, submat);
		}
	}
	float det = 0;
	for (uint8_t i = 0; i < rows; i++)
		det = (i % 2) ? det - mat[0][i] * minors[0][i] : det + mat[0][i] * minors[0][i];

	// calculate cofactors and adjoint
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			if ((i + j) % 2) minors[i][j] *= -1;

	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[i][j] = minors[j][i] / det;
}

float Mat_Det(uint8_t rows, float mat[rows][rows]) {
	if (rows == 2) return mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
	float det = 0;
	for (uint8_t i = 0; i < rows; i++) {
		// determine submatrix and its determinant
		float submat[rows-1][rows-1];
		for (uint8_t j = 0; j < rows-1; j++) {
			uint8_t offsetK = 0;
			for (uint8_t k = 0; k < rows-1; k++) {
				if (k == i) offsetK = 1;
				submat[j][k] = mat[j+1][k+offsetK];
			}
		}
		float submat_det = Mat_Det(rows-1, submat);
		det = (i % 2) ? det - mat[0][i]*submat_det : det + mat[0][i]*submat_det;
	}
	return det;
}


void Mat_Print(uint8_t rows, uint8_t cols, float mat[rows][cols], char * title) {
	char msg[MSG_LEN];
	snprintf(msg, MSG_LEN, "%s: {", title);
	for (uint8_t i = 0; i < rows; i++) {
		snprintf(msg, MSG_LEN, "%s{", msg);
		for (uint8_t j = 0; j < cols; j++) {
			snprintf(msg, MSG_LEN, "%s%.3f, ", msg, mat[i][j]);
		}
		snprintf(msg, MSG_LEN, "%s},\r\n", msg);
	}
	snprintf(msg, MSG_LEN, "%s}\r\n", msg);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 0x100);
}

void Vec_Print(uint8_t cols, float vec[cols], char * title) {
	char msg[MSG_LEN];
	snprintf(msg, MSG_LEN, "%s: {", title);
	for (uint8_t i = 0; i < cols; i++)
		snprintf(msg, MSG_LEN, "%s%.3f, ", msg, vec[i]);
	snprintf(msg, MSG_LEN, "%s}\r\n", msg);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 0x100);
}

/* Quaternion modifications */
void q_mul(float result[4], float q1[4], float q2[4]) {
	result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void q_inv(float result[4], float q[4]) {
	float div = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	result[0] = q[0] / div;
	result[1] = -q[1] / div;
	result[2] = -q[2] / div;
	result[3] = -q[3] / div;
}

void q_rot(float result[4], float q1[4], float q2[4]) {
	float qInv[4], qInt[4];
	q_inv(qInv, q1);
	q_mul(qInt, q2, qInv);
	q_mul(result, q1, qInt);
}

void q_to_r(float r[3], float q[4]) {
	float a = 2.0 * acos(q[0]);
	if (a == 0) {
		r[0] = 0.0;
		r[1] = 0.0;
		r[2] = 0.0;
	} else {
		float scale = a / sin(a / 2.0);
		r[0] = scale * q[1];
		r[1] = scale * q[2];
		r[2] = scale * q[3];
	}
}

void r_to_q(float q[4], float r[3]) {
	float a = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
	if (a == 0) {
		q[0] = 1.0;
		q[1] = 0.0;
		q[2] = 0.0;
		q[3] = 0.0;
	} else {
		q[0] = cos(a / 2.0);
		q[1] = (r[0] / a) * sin(a / 2.0);
		q[1] = (r[1] / a) * sin(a / 2.0);
		q[1] = (r[2] / a) * sin(a / 2.0);
	}
}

void q_to_e(float e[3], float q[4]) {
	e[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
	e[1] = asin(2*(q[0]*q[2] - q[3]*q[1]));
	e[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
}

void q_to_R(float R[3][3], float q[4]) {
	R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	R[0][1] = 2.0 * (q[1]*q[2] - q[0]*q[3]);
	R[0][2] = 2.0 * (q[1]*q[3] + q[0]*q[2]);
	R[1][0] = 2.0 * (q[1]*q[2] + q[0]*q[3]);
	R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	R[1][2] = 2.0 * (q[2]*q[3] - q[0]*q[1]);
	R[2][0] = 2.0 * (q[1]*q[3] - q[0]*q[2]);
	R[2][1] = 2.0 * (q[2]*q[3] + q[0]*q[1]);
	R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
}

void norm(uint8_t len, float x[len]) {
	float sum = 0;
	for (uint8_t i = 0; i < len; i++) sum += x[i]*x[i];
	sum = sqrt(sum);
	for (uint8_t i = 0; i < len; i++) x[i] /= sum;
}

void cholesky(uint8_t rows, float L[rows][rows], float A[rows][rows]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			L[i][j] = 0;

	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < i+1; j++) {
			float sum = 0;
			for (uint8_t k = 0; k < j; k++) sum += L[i][k] * L[j][k];
			L[i][j] = (i == j) ? sqrt(A[i][j] - sum) : 1 / L[j][j] * (A[i][j] - sum);
		}
	}
}
