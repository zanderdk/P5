/*
 * MatrixAlgebra.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void MatrixCopy(int columns, int row, double [columns][row], double [columns][row]);
void MatrixAddition(int row, int coulom, double [coulom][row], double [coulom][row], double [coulom][row]);
void SkalarMultiplikation(int row, int coulom, double [coulom][row], int multiplier, double [coulom][row]);
S8 MatrixMultiplikation(int columnsFirst, int rowFirst, int columnsSecond, int rowSecond, double [columnsFirst][rowFirst], double [columnsSecond][rowSecond], double [columnsFirst][rowSecond]);
void MatrixTranspose(int columns, int row, double [columns][row], double [columns][row]);
S8 MatrixInvers(int row, int columns, double [columns][row], double [columns][row]);
