/*
 * MatrixAlgebra.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void matrixCopy(S32 row, S32 columns, double [row][columns], double [row][columns]);
void matrixAddition(S32 row, S32 columns, double [row][columns], double [row][columns], double [row][columns]);
void skalarMultiplikation(S32 row, S32 columns, double [row][columns], double multiplier, double [row][columns]);
S8 matrixMultiplikation(S32 rowFirst, S32 columnsFirst, S32 rowSecond, S32 columnsSecond, double [rowFirst][columnsFirst], double [rowSecond][columnsSecond], double [rowFirst][columnsSecond]);
void matrixTranspose(S32 row, S32 columns, double [row][columns], double [row][columns]);
S8 matrixInvers(S32 row, S32 columns, double [columns][row], double [columns][row]);
