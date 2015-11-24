#include "ecrobot_types.h"

void matrixCopy(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]){
	U8 i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			out[i][j] = in[i][j];
		}
	}
}

void matrixAdd(U8 rows, U8 cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]){
	double tempA[rows][cols];
	double tempB[rows][cols];
	matrixCopy(rows, cols, a, tempA);
	matrixCopy(rows, cols, b, tempB);
	U8 i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			out[i][j] = tempA[i][j] + tempB[i][j];
		}
	}
}

void matrixSubtract(U8 rows, U8 cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]){
	double tempA[rows][cols];
	double tempB[rows][cols];
	matrixCopy(rows, cols, a, tempA);
	matrixCopy(rows, cols, b, tempB);
	U8 i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			out[i][j] = tempA[i][j] - tempB[i][j];
		}
	}
}

void matrixScale(U8 rows, U8 cols, double in[rows][cols], double scalar, double out[rows][cols]){
	double temp[rows][cols];
	matrixCopy(rows, cols, in, temp);
	U8 i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			out[i][j] = temp[i][j] * scalar;
		}
	}
}

U8 matrixMultiply(U8 rowsA, U8 colsA, U8 rowsB, U8 colsB, double a[rowsA][colsA],
	                    double b[rowsB][colsB], double out[rowsA][colsB]){
	if(colsA != rowsB)
		return -1;

	double tempA[rowsA][colsA];
	double tempB[rowsB][colsB];
	matrixCopy(rowsA, colsA, a, tempA);
	matrixCopy(rowsB, colsB, b, tempB);
	double sum = 0;
	U8 i, j, k;
	for(i = 0; i < rowsA; i++){
		for(j = 0; j < colsB; j++){
			for(k = 0; k < colsA; k++){
				sum += tempA[i][k]*tempB[k][j];
			}
			out[i][j] = sum;
			sum = 0;
		}
	}
	return 1;
}

void matrixTranspose(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]){
	double temp[rows][cols];
	matrixCopy(rows, cols, in, temp);
	U8 i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			out[j][i] = temp[i][j];
		}
	}
}

/* Inverts a 2-by-2 matrix */
U8 matrixInvert(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]){
	if(rows != 2 || cols != 2)
		return -1;

	double temp[rows][cols];
	matrixCopy(rows, cols, in, temp);

	double determinant = temp[0][0] * temp[1][1] - temp[0][1] * temp[1][0];
	if(determinant == 0) 
		return -1;
	
	out[0][0] = temp[1][1];
	out[0][1] = -temp[0][1];
	out[1][0] = -temp[1][0];
	out[1][1] = temp[0][0];
	matrixScale(rows, cols, out, 1.0 / determinant, out);
	return 1;
}

long double matrixDeterminant(U8 rows, U8 cols, double in[rows][cols]){
	if(rows != 2 || cols != 2)
		return -1;
	
	return in[0][0] * in[1][1] - in[0][1] * in[1][0];
}

/* Multiplies a 1-by-n matrix with a n-by-1 matrix, resulting in a scalar value */
double matrixMultiplyToScalar(U8 rowsA, U8 colsA, U8 rowsB, U8 colsB, double a[][colsA], double b[][colsB]){
	if(rowsA != 1 || colsB != 1 || colsA != rowsB)
		return -1;

	U8 i;
	double sum = 0;
	for(i = 0; i < colsA; i++)
		sum += a[0][i] * b[i][0];
	return sum;
}
