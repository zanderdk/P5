/*
 * MatrixAlgebra.c
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void MatrixAddition(int row, int columns, double firstMatrix[columns][row], double secondMatrix[columns][row], double rtnMatrix[columns][row])
{
	int i;
	int j;
	for(i=0; i<columns; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
		}
	}
}

void SkalarMultiplikation(int row, int columns, double matrix[columns][row], int multiplier, double rtnMatrix[columns][row])
{
		int i = 0;
		int j = 0;
		for(i=0; i<columns; i++)
		{
			for(j=0; j<row; j++)
			{
				rtnMatrix[i][j] = matrix[i][j] * multiplier;
			}
		}

}

S8 MatrixMultiplikation(int rowFirst, int columnsFirst, int rowSecond, int columnsSecond, double firstMatrix[columnsFirst][rowFirst], double secondMatrix[columnsSecond][rowSecond], double rtnMatrix[columnsFirst][rowSecond])
{
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
					sum += firstMatrix[i][k]*secondMatrix[k][j];
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

void MatrixTranspose(int row, int columns, double Matrix[columns][row], double rtnMatrix[columns][row])
{
	int i = 0;
	int j = 0;
	for(i=0; i<columns; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[j][i] = Matrix[i][j];
		}
	}
}


S8 MatrixInvers(int row, int columns, double Matrix[columns][row], double rtnMatrix[columns][row])
{
	if(row == 2 && columns == 2)
	{
		rtnMatrix[0][0] = Matrix[1][1];
		rtnMatrix[0][1] = -Matrix[1][0];
		rtnMatrix[1][0] = -Matrix[0][1];
		rtnMatrix[1][1] = Matrix[0][0];
		return 1;
	}
	else
	{
		return -1;
	}
}
