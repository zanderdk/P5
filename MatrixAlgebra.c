#include "ecrobot_types.h"

void MatrixCopy(S32 row, S32 columns, double rtnMatrix[row][columns], double Matrix[row][columns])
{
	S32 i = 0;
	S32 j = 0;
	for(i=0; i < row; i++)
	{
		for(j=0; j < columns; j++)
		{
			rtnMatrix[i][j] = Matrix[i][j];
		}
	}
}

void MatrixAddition(S32 row, S32 columns, double firstMatrix[row][columns], double secondMatrix[row][columns], double rtnMatrix[row][columns])
{
	double firstMatrixTmp[row][columns];
	double secondMatrixTmp[row][columns];
	MatrixCopy(row, columns, firstMatrixTmp, firstMatrix);
	MatrixCopy(row, columns, secondMatrixTmp, secondMatrix);
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

void SkalarMultiplikation(S32 row, S32 columns, double matrix[row][columns], S32 multiplier, double rtnMatrix[row][columns])
{
		double matrixTmp[row][columns];
		MatrixCopy(row, columns, matrixTmp, matrix);
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

S8 MatrixMultiplikation(S32 rowFirst, S32 columnsFirst, S32 rowSecond, S32 columnsSecond, double firstMatrix[rowFirst][columnsFirst], double secondMatrix[rowSecond][columnsSecond], double rtnMatrix[rowFirst][columnsSecond])
{
	double firstMatrixTmp[rowFirst][columnsFirst];
	double secondMatrixTmp[rowSecond][columnsSecond];
	MatrixCopy(rowFirst, columnsFirst, firstMatrixTmp, firstMatrix);
	MatrixCopy(rowSecond, columnsSecond, secondMatrixTmp, secondMatrix);
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

void MatrixTranspose(S32 row, S32 columns, double Matrix[row][columns], double rtnMatrix[row][columns])
{
	double MatrixTmp[row][columns];
	MatrixCopy(row, columns, MatrixTmp, Matrix);
	S32 i = 0;
	S32 j = 0;
	for(i=0; i<row; i++)
	{
		for(j=0; j<columns; j++)
		{
			rtnMatrix[j][i] = MatrixTmp[i][j];
		}
	}
}


S8 MatrixInvers(S32 row, S32 columns, double Matrix[columns][row], double rtnMatrix[columns][row])
{
	double MatrixTmp[columns][row];
	MatrixCopy(columns, row, MatrixTmp, Matrix);
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
