/*
 * MatrixAlgebra.c
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void MatrixAddition(int columns, int row, double firstMatrix[columns][row], double secondMatrix[columns][row], double rtnMatrix[columns][row])
{
	double firstMatrixTmp[columns][row] = firstMatrix;
	double secondMatrixTmp[columns][row] = secondMatrix;
	int i;
	int j;
	for(i=0; i<columns; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[i][j] = firstMatrixTmp[i][j] + secondMatrixTmp[i][j];
		}
	}
}

void SkalarMultiplikation(int columns, int row, double matrix[columns][row], int multiplier, double rtnMatrix[columns][row])
{
		double matrixTmp[columns][row] = matrix;
		int i = 0;
		int j = 0;
		for(i=0; i<columns; i++)
		{
			for(j=0; j<row; j++)
			{
				rtnMatrix[i][j] = matrixTmp[i][j] * multiplier;
			}
		}

}

S8 MatrixMultiplikation(int columnsFirst, int rowFirst, int columnsSecond, int rowSecond, double firstMatrix[columnsFirst][rowFirst], double secondMatrix[columnsSecond][rowSecond], double rtnMatrix[columnsFirst][rowSecond])
{
	double firstMatrixTmp[columnsFirst][rowFirst] = firstMatrix;
	double  double secondMatrixTmp[columnsSecond][rowSecond] = secondMatrix;
	double sum = 0;
	int i;
	int j;
	int k;
	if(rowFirst == columnsSecond)
	{
		for(i=0; i<columnsFirst; i++)
		{
			for(j=0; j<rowSecond; j++)
			{
				for(k=0; k<columnsSecond; k++)
				{
					sum += firstMatrixTmp[i][k]*secondMatrixTmp[k][j];
				}
			}
		}
	}
	else
	{
		return -1;
	}
	return 1;

}

void MatrixTranspose(int columns, int row, double Matrix[columns][row], double rtnMatrix[columns][row])
{
	double MatrixTmp[columns][row] = Matrix;
	int i = 0;
	int j = 0;
	for(i=0; i<columns; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[j][i] = MatrixTmp[i][j];
		}
	}
}


S8 MatrixInvers(int row, int columns, double Matrix[columns][row], double rtnMatrix[columns][row])
{
	double MatrixTmp[columns][row] = Matrix;
	if(row == 2 && columns == 2)
	{
		rtnMatrix[0][0] = MatrixTmp[1][1];
		rtnMatrix[0][1] = -MatrixTmp[1][0];
		rtnMatrix[1][0] = -MatrixTmp[0][1];
		rtnMatrix[1][1] = MatrixTmp[0][0];
		return 1;
	}
	else
	{
		return -1;
	}
}
