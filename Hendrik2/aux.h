#ifndef AUX_H_
#define AUX_H_

#include <stdlib.h>
#include <stdint-gcc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MSG_LEN 500

// Matrix equations
void Mat_T(uint8_t rows, uint8_t cols, float result[cols][rows], float mat[rows][cols]);
void Mat_Add(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]);
void Mat_Sub(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]);
void Mat_Mul(uint8_t rows, uint8_t cols, uint8_t add, float result[rows][cols], float mat1[rows][add], float mat2[add][cols]);
void Mat_Inv(uint8_t rows, float result[rows][rows], float mat[rows][rows]);
float Mat_Det(uint8_t rows, float mat[rows][rows]);
void Mat_Print(uint8_t rows, uint8_t cols, float mat[rows][cols], char * title);
void Vec_Print(uint8_t cols, float vec[cols], char * title);

// Quaternion modifications
void q_mul(float result[4], float q1[4], float q2[4]);
void q_inv(float result[4], float q[4]);
void q_rot(float result[4], float q1[4], float q2[4]);
void q_to_r(float r[3], float q[4]);
void r_to_q(float q[4], float r[3]);
void q_to_e(float e[3], float q[4]);
void q_to_R(float R[3][3], float q[4]);
void norm(uint8_t len, float x[len]);
void cholesky(uint8_t rows, float L[rows][rows], float A[rows][rows]);

#endif /* AUX_H_ */
