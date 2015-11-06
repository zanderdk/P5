/*
 * MatrixAlgebra.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void matrixCopy(int columns, int row, double [columns][row], double [columns][row]);
void matrixAddition(int row, int coulom, double [coulom][row], double [coulom][row], double [coulom][row]);
void skalarMultiplikation(int row, int coulom, double [coulom][row], int multiplier, double [coulom][row]);
S8 matrixMultiplikation(int columnsFirst, int rowFirst, int columnsSecond, int rowSecond, double [columnsFirst][rowFirst], double [columnsSecond][rowSecond], double [columnsFirst][rowSecond]);
void matrixTranspose(int columns, int row, double [columns][row], double [columns][row]);
S8 matrixInvers(int row, int columns, double [columns][row], double [columns][row]);
