/*
 * MatrixAlgebra.c
 *
 *  Created on: Nov 5, 2015
 *      Author: Kenneth
 */

#include "ecrobot_types.h"

void MatrixAddition(int row, int coulom, double firstMatrix[coulom][row], double secondMatrix[coulom][row], double rtnMatrix[coulom][row])
{
	int i;
	int j;
	for(i=0; i<coulom; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[i][j] = firstMatrix[i][j] + secondMatrix[i][j];
		}
	}
}

void SkalarMultiplikation(int row, int coulom, double matrix[coulom][row], int multiplier, double rtnMatrix[coulom][row])
{
		int i = 0;
		int j = 0;
		for(i=0; i<coulom; i++)
		{
			for(j=0; j<row; j++)
			{
				rtnMatrix[i][j] = matrix[i][j] * multiplier;
			}
		}

}

S8 MatrixMultiplikation(int rowFirst, int coulomFirst, int rowSecond, int coulomSecond, double firstMatrix[coulomFirst][rowFirst], double secondMatrix[coulomSecond][rowSecond], double rtnMatrix[coulomFirst][rowSecond])
{
	double sum = 0;
	int i;
	int j;
	int k;
	if(rowFirst == coulomSecond)
	{
		for(i=0; i<coulomFirst; i++)
		{
			for(j=0; j<rowSecond; j++)
			{
				for(k=0; k<coulomSecond; k++)
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

void MatrixTranspose(int row, int coulom, double Matrix[coulom][row], double rtnMatrix[coulom][row])
{
	int i = 0;
	int j = 0;
	for(i=0; i<coulom; i++)
	{
		for(j=0; j<row; j++)
		{
			rtnMatrix[j][i] = Matrix[i][j];
		}
	}
}


void MatrixInvers(int row, int coulom, double Matrix[coulom][row], double rtnMatrix[coulom][row])
{

}
