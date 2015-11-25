#include "ecrobot_types.h"

void matrixCopy(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]);
void matrixAdd(U8 rows, U8 cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]);
void matrixSubtract(U8 rows, U8 cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]);
void matrixScale(U8 rows, U8 cols, double in[rows][cols], double scalar, double out[rows][cols]);
U8 matrixMultiply(U8 rowsA, U8 colsA, U8 rowsB, U8 colsB, double a[rowsA][colsA],
	                    double b[rowsB][colsB], double out[rowsA][colsB]);
void matrixTranspose(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]);
U8 matrixInvert(U8 rows, U8 cols, double in[rows][cols], double out[rows][cols]);
long double matrixDeterminant(U8 rows, U8 cols, double in[rows][cols]);
double matrixMultiplyToScalar(U8 rowsA, U8 colsA, U8 rowsB, U8 colsB, double a[][colsA], double b[][colsB]);
