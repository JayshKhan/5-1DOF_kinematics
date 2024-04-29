 //
// Created by Jaysh Khan on 4/21/24.
//

#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include"./constants.h"
/**
 * Transformation Matrix Initializer
 * @param alpha[], theta[], a[], d[] arrays of parameters
 * @return array of 4x4 transformation matrix from T_0_1 to T_4-5
 * @author Jaysh Khan
 */

Matrix4x4 * T_matrices(float alpha[], int theta[], float a[], float d[]) {
  Matrix4x4 * T = new Matrix4x4[SERVO_COUNT]; // Transformation matrix, T1, T2, T3, T4, T5
  for (int i = 1; i < SERVO_COUNT; i++) {
    T[i].name = "T_"  + String(i - 1) + "_" + String(i);

    T[i].data[0][0] = cos(theta[i]);
    T[i].data[0][1] = -sin(theta[i]);
    T[i].data[0][2] = 0;
    T[i].data[0][3] = a[i - 1];

    T[i].data[1][0] = sin(theta[i]) * cos(alpha[i - 1]);
    T[i].data[1][1] = cos(theta[i]) * cos(alpha[i - 1]);
    T[i].data[1][2] = -sin(alpha[i - 1]);
    T[i].data[1][3] = -sin(alpha[i - 1]) * d[i];

    T[i].data[2][0] = sin(theta[i]) * sin(alpha[i - 1]);
    T[i].data[2][1] = cos(theta[i]) * sin(alpha[i - 1]);
    T[i].data[2][2] = cos(alpha[i - 1]);
    T[i].data[2][3] = cos(alpha[i - 1]) * d[i];

    T[i].data[3][0] = 0;
    T[i].data[3][1] = 0;
    T[i].data[3][2] = 0;
    T[i].data[3][3] = 1;

  }
  return T;
  //TODO: Free the memory
}

/**
 * Forward Kinematics
 * @param T[] array of 4x4 transformation matrix from T_0_1 to T_4-5
 * @return 4x4 transformation matrix
 * @author Jaysh Khan
 */

Matrix4x4 forwardKinematics(Matrix4x4 T[]) {
  Matrix4x4 T_0_5 = Matrix4x4();
  for (int i = 1; i < SERVO_COUNT; i++) {
    T_0_5 = Multiply(T_0_5, T[i]);
  }
  return T_0_5;
}

/**
 * Matrix Multiplication
 * @param Matrix4x4 A, Matrix4x4 B ,m, n, p, Matrix4x4 C
 * @return C = A*B  where A is m x n and B is n x p
 * @author Jaysh Khan
 */

Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B) {
  int m = 4, n = 4, p = 4;
  //TODO Add Check for invalid dimensions
  Matrix4x4 C;
  int i, j, k;
  for (i = 0; i < m; i++)
    for (j = 0; j < n; j++) {
      C.data[i][j] = 0;
      for (k = 0; k < p; k++)
        C.data[i][j] = C.data[i][j] + A.data[i][k] * B.data[k][j];
    }
  return C;
}

#endif //FORWARDKINEMATICS_H
