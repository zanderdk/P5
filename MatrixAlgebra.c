#include "ecrobot_types.h"

void matrixCopy(S32 row, S32 columns, double rtnmatrix[row][columns], double matrix[row][columns])
{
	S32 i = 0;
	S32 j = 0;
	for(i=0; i < row; i++)
	{
		for(j=0; j < columns; j++)
		{
			rtnmatrix[i][j] = matrix[i][j];
		}
	}
}

void matrixAddition(S32 row, S32 columns, double firstmatrix[row][columns], double secondmatrix[row][columns], double rtnmatrix[row][columns])
{
	double firstmatrixTmp[row][columns];
	double secondmatrixTmp[row][columns];
	matrixCopy(row, columns, firstmatrixTmp, firstmatrix);
	matrixCopy(row, columns, secondmatrixTmp, secondmatrix);
	S32 i = 0;
	S32 j = 0;
	for(i=0; i<row; i++)
	{
		for(j=0; j<columns; j++)
		{
			rtnmatrix[i][j] = firstmatrixTmp[i][j] + secondmatrixTmp[i][j];
		}
	}
}

void skalarMultiplikation(S32 row, S32 columns, double matrix[row][columns], S32 multiplier, double rtnmatrix[row][columns])
{
		double matrixTmp[row][columns];
		matrixCopy(row, columns, matrixTmp, matrix);
		S32 i = 0;
		S32 j = 0;
		for(i=0; i<row; i++)
		{
			for(j=0; j<columns; j++)
			{
				rtnmatrix[i][j] = matrixTmp[i][j] * multiplier;
			}
		}

}

S8 matrixMultiplikation(S32 rowFirst, S32 columnsFirst, S32 rowSecond, S32 columnsSecond, double firstmatrix[rowFirst][columnsFirst], double secondmatrix[rowSecond][columnsSecond], double rtnmatrix[rowFirst][columnsSecond])
{
	double firstmatrixTmp[rowFirst][columnsFirst];
	double secondmatrixTmp[rowSecond][columnsSecond];
	matrixCopy(rowFirst, columnsFirst, firstmatrixTmp, firstmatrix);
	matrixCopy(rowSecond, columnsSecond, secondmatrixTmp, secondmatrix);
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
					sum += firstmatrixTmp[i][k]*secondmatrixTmp[k][j];
				}
				rtnmatrix[i][j] = sum;
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

void matrixTranspose(S32 row, S32 columns, double matrix[row][columns], double rtnmatrix[row][columns])
{
	double matrixTmp[row][columns];
	matrixCopy(row, columns, matrixTmp, matrix);
	S32 i = 0;
	S32 j = 0;
	for(i=0; i<row; i++)
	{
		for(j=0; j<columns; j++)
		{
			rtnmatrix[j][i] = matrixTmp[i][j];
		}
	}
}


S8 matrixInvers(S32 row, S32 columns, double matrix[columns][row], double rtnmatrix[columns][row])
{
	double matrixTmp[columns][row];
	matrixCopy(columns, row, matrixTmp, matrix);
	if(row == 2 && columns == 2)
	{
		rtnmatrix[0][0] = matrixTmp[1][1];
		rtnmatrix[0][1] = -matrixTmp[0][1];
		rtnmatrix[1][0] = -matrixTmp[1][0];
		rtnmatrix[1][1] = matrixTmp[0][0];
		return 1;
	}
	else
	{
		return -1;
	}
}
