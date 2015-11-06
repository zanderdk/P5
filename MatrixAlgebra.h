/*
 * MatrixAlgebra.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void MatrixAddition(int row, int coulom, double [coulom][row], double [coulom][row], double [coulom][row]);
void SkalarMultiplikation(int row, int coulom, double [coulom][row], int multiplier, double [coulom][row]);
void MatrixMultiplikation(int row, int coulom, double [coulom][row], double [coulom][row], double [coulom][row]);
void MatrixTranspose(int row, int coulom, double [coulom][row], double [coulom][row]);
void MatrixInvers(int row, int coulom, double [coulom][row], double [coulom][row]);
