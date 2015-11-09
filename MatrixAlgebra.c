#include "ecrobot_types.h"

void matrixCopy(S32 row, S32 columns, double rtnMatrix[row][columns], double matrix[row][columns])
{
	S32 i = 0;
	S32 j = 0;
	for(i=0; i < row; i++)
	{
		for(j=0; j < columns; j++)
		{
			rtnMatrix[i][j] = matrix[i][j];
		}
	}
}

void matrixAddition(S32 row, S32 columns, double firstMatrix[row][columns], double secondMatrix[row][columns], double rtnMatrix[row][columns])
{
	double firstMatrixTmp[row][columns];
	double secondMatrixTmp[row][columns];
	matrixCopy(row, columns, firstMatrixTmp, firstMatrix);
	matrixCopy(row, columns, secondMatrixTmp, secondMatrix);
	S32 i = 0;
	S32 j = 0;
	for(i=0; i<row; i++)
	{
		for(j=0; j<columns; j++)
		{
			rtnMatrix[i][j] = firstMatrixTmp[i][j] + secondMatrixTmp[i][j];
		}
	}
}

void skalarMultiplikation(S32 row, S32 columns, double matrix[row][columns], double multiplier, double rtnMatrix[row][columns])
{
		double matrixTmp[row][columns];
		matrixCopy(row, columns, matrixTmp, matrix);
		S32 i = 0;
		S32 j = 0;
		for(i=0; i<row; i++)
		{
			for(j=0; j<columns; j++)
			{
				rtnMatrix[i][j] = matrixTmp[i][j] * multiplier;
			}
		}

}

U8 matrixMultiplikation(S32 rowFirst, S32 columnsFirst, S32 rowSecond, S32 columnsSecond, double firstMatrix[rowFirst][columnsFirst], double secondMatrix[rowSecond][columnsSecond], double rtnMatrix[rowFirst][columnsSecond])
{
	double firstMatrixTmp[rowFirst][columnsFirst];
	double secondMatrixTmp[rowSecond][columnsSecond];
	matrixCopy(rowFirst, columnsFirst, firstMatrixTmp, firstMatrix);
	matrixCopy(rowSecond, columnsSecond, secondMatrixTmp, secondMatrix);
	double sum = 0;
	S32 i = 0;
	S32 j = 0;
	S32 k = 0;
	if(columnsFirst == rowSecond)
	{
		for(i=0; i<rowFirst; i++)
		{
			for(j=0; j<columnsSecond; j++)
			{
				for(k=0; k<rowSecond; k++)
				{
					sum += firstMatrixTmp[i][k]*secondMatrixTmp[k][j];
				}
				rtnMatrix[i][j] = sum;
				sum = 0;
			}
		}
	}
	else
	{
		return -1;
	}
	return 1;

}

void matrixTranspose(S32 row, S32 columns, double matrix[row][columns], double rtnMatrix[row][columns])
{
	double matrixTmp[row][columns];
	matrixCopy(row, columns, matrixTmp, matrix);
	S32 i = 0;
	S32 j = 0;
	for(i=0; i<row; i++)
	{
		for(j=0; j<columns; j++)
		{
			rtnMatrix[j][i] = matrixTmp[i][j];
		}
	}
}


U8 matrixInvers(S32 row, S32 columns, double matrix[row][columns], double rtnMatrix[row][columns])
{
	double matrixTmp[row][columns];
	matrixCopy(row, columns, matrixTmp, matrix);
	if(row == 2 && columns == 2)
	{
		rtnMatrix[0][0] = matrixTmp[1][1];
		rtnMatrix[0][1] = -matrixTmp[0][1];
		rtnMatrix[1][0] = -matrixTmp[1][0];
		rtnMatrix[1][1] = matrixTmp[0][0];
		
		double determinant = 1.0/(matrixTmp[0][0]*matrixTmp[1][1]-matrixTmp[0][1]*matrixTmp[1][0]);
		if(determinant == 0) return -1;
		skalarMultiplikation(row, columns, rtnMatrix, determinant, rtnMatrix);
		return 1;
	}
	else
	{
		return -1;
	}
}

long double matrixDeterminant(S32 row, S32 columns, double matrix[row][columns])
{
	if(row == 2 && columns == 2)
	{
		return matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0];
	}
	else
	{
		return -1;
	}
}


