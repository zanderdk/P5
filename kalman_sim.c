#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DELTA_T 0.01
#define READINGS 1000
#define POS_START 0
#define SPEED 50
#define VK 8.0

/* Function prototypes */
void kalman(double zn);
double random_error(double range);
void matrixCopy(int rows, int cols, double in[rows][cols], double out[rows][cols]);
void matrixAdd(int rows, int cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]);
void matrixSubtract(int rows, int cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]);
void matrixScale(int rows, int cols, double in[rows][cols], double scalar, double out[rows][cols]);
int matrixMultiply(int rowsA, int colsA, int rowsB, int colsB, double a[rowsA][colsA],
                        double b[rowsB][colsB], double out[rowsA][colsB]);
void matrixTranspose(int rows, int cols, double in[rows][cols], double out[rows][cols]);
int matrixInvert(int rows, int cols, double in[rows][cols], double out[rows][cols]);
long double matrixDeterminant(int rows, int cols, double in[rows][cols]);
double matrixMultiplyToScalar(int rowsA, int colsA, int rowsB, int colsB, double a[][colsA], double b[][colsB]);

/* Global variables */
double x[2][1] = {{ -40.0}, {70.0}};
double P[2][2] = {{0.4, 0.0}, {0.0, 30.0}};
double K[2][1] = {{0.0}, {0.0}};
double x_prior[2][1] = {{0.0}, {0.0}};
double P_prior[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
double measurement_error;


/* Main function taking 4 optional arguments: 
   arg1: Number of loops                    (int)
   arg2: Variance in measurements           (double)
   arg3: Speed of target in deg/sec         (double)
   arg4: Initial position of target in deg  (double) */
int main(int argc, char* argv[]){
    srand(time(NULL));      // Random seed

    /* Check for number of loops argument */
    if(argc > 5){
        printf("Too many arguments!");
        exit(EXIT_FAILURE);
    }
    int loops = (argc > 1 ? atoi(argv[1]) : READINGS);
    measurement_error = (argc > 2 ? atof(argv[2]) : VK);
    double speed_actual = (argc > 3 ? atof(argv[3]) : SPEED);
    double pos_actual = (argc > 4 ? atof(argv[4]) : POS_START);

    /* Open the file used for output */
    FILE *f = fopen("kalman_data.csv", "w");
    printf("FILE OPENED!\n");

    if(f == NULL){
        printf("Error opening file!\n");
        exit(EXIT_FAILURE);
    }
    /* Print column labels */
    fprintf(f, "timestep, ms, pos_actual, speed_actual, measurement, error, pos_pred, "
        "speed_pred, cov1, cov2, cov3, cov4, gain1, gain2, pos_pred_upd, speed_pred_upd, "
        "cov_upd1, cov_upd2, cov_upd3, cov_upd4\n");

    /* Simulate the kalman filter iteratively */
    int i;
    for(i = 0; i <= loops; i++){
        /* Actual data */
        int timestep = i;
        int time_ms = timestep * (DELTA_T * 1000);
        double delta_time_sec = (i == 0 ? 0.0 : DELTA_T);
        pos_actual = pos_actual + speed_actual * delta_time_sec;

        /* Perform measurement */
        double error = (i == 0 ? 0.0 : random_error(measurement_error));
        double measurement = pos_actual + error;
        /* Save priori state and covariance */
        matrixCopy(2, 1, x, x_prior);
        matrixCopy(2, 2, P, P_prior);
        /* Run Kalman filter for timestep k > 0 */
        if(i > 0)
            kalman(measurement);

        /* Values calculated by the kalman filter */
        double pos_pred = x_prior[0][0];
        double speed_pred = x_prior[1][0];
        double cov_1 = P_prior[0][0];
        double cov_2 = P_prior[0][1];
        double cov_3 = P_prior[1][0];
        double cov_4 = P_prior[1][1];
        double kg_1 = K[0][0];
        double kg_2 = K[1][0];
        double pos_pred_upd = x[0][0];
        double speed_pred_upd = x[1][0];
        double cov_upd_1 = P[0][0];
        double cov_upd_2 = P[0][1];
        double cov_upd_3 = P[1][0];
        double cov_upd_4 = P[1][1];

        /* Print the data for each timestep in a csv file */
        fprintf(f, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            timestep,
            time_ms,
            pos_actual,
            speed_actual,
            measurement,
            error,
            pos_pred,
            speed_pred,
            cov_1,
            cov_2,
            cov_3,
            cov_4,
            kg_1,
            kg_2,
            pos_pred_upd,
            speed_pred_upd,
            cov_upd_1,
            cov_upd_2,
            cov_upd_3,
            cov_upd_4);
    }
    printf("ITERATIONS: %d!\n", i - 1);

    /* Close the file again */
    fclose(f);
    printf("FILE CLOSED!\n");

    return 0;
}

/* Generate random variable in given range, based on variance in measurement */
double random_error(double range){
    return (range / 2) - (double)(rand() % (((int)(measurement_error * 1000.0)) + 1)) / 1000.0;
}

/*  Kalman filter identical to the NXT version, except global K matrix */
void kalman(double zn) {
    /* Constants and matrices used for kalman calculations */
    double R = measurement_error;
    static double I[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
    static double t = 0.01;
    double h[1][2] = {{1.0, 0.0}};
    double hT[2][1] = {{1.0}, {0.0}};
    double a[2][2] = {{1.0, t}, {0.0, 1.0}};
    double aT[2][2] = {{1.0, 0.0}, {t, 1.0}};
    double y[2][1] = {{0.0}, {0.0}};
    double temp[1][2] = {{0.0, 0.0}};
    double temp2v = 0.0;
    double (*temp2)[1] = (double ( *)[1])(&temp2v);
    double temp3[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

    /* Calculate kalman gain in K */
    matrixMultiply(1, 2, 2, 2, h, P, temp);
    matrixMultiply(1, 2, 2, 1, temp, hT, temp2);
    temp2v += R;
    temp2v = 1.0 / temp2v;
    matrixMultiply(2, 2, 2, 1, P, hT, K);
    matrixScale(2, 1, K, temp2v, K);

    /* Update state estimate */
    matrixMultiply(1, 2, 2, 1, h, x, temp2);
    zn -= temp2v;
    matrixScale(2, 1, K, zn, y);
    matrixAdd(2, 1, x, y, x);

    /* Update state covariance */
    matrixMultiply(2, 1, 1, 2, K, h, temp3);
    matrixSubtract(2, 2, I, temp3, temp3);
    matrixMultiply(2, 2, 2, 2, temp3, P, P);

    /* Project into timestep k+1 */
    matrixMultiply(2, 2, 2, 1, a, x, x);
    matrixMultiply(2, 2, 2, 2, a, P, P);
    matrixMultiply(2, 2, 2, 2, P, aT, P);
}


/* Matrix functions from the NXT version, with U8 changed int */
void matrixCopy(int rows, int cols, double in[rows][cols], double out[rows][cols]){
    int i, j;
    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            out[i][j] = in[i][j];
        }
    }
}

void matrixAdd(int rows, int cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]){
    double tempA[rows][cols];
    double tempB[rows][cols];
    matrixCopy(rows, cols, a, tempA);
    matrixCopy(rows, cols, b, tempB);
    int i, j;
    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            out[i][j] = tempA[i][j] + tempB[i][j];
        }
    }
}

void matrixSubtract(int rows, int cols, double a[rows][cols], double b[rows][cols], double out[rows][cols]){
    double tempA[rows][cols];
    double tempB[rows][cols];
    matrixCopy(rows, cols, a, tempA);
    matrixCopy(rows, cols, b, tempB);
    int i, j;
    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            out[i][j] = tempA[i][j] - tempB[i][j];
        }
    }
}

void matrixScale(int rows, int cols, double in[rows][cols], double scalar, double out[rows][cols]){
    double temp[rows][cols];
    matrixCopy(rows, cols, in, temp);
    int i, j;
    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            out[i][j] = temp[i][j] * scalar;
        }
    }
}

int matrixMultiply(int rowsA, int colsA, int rowsB, int colsB, double a[rowsA][colsA],
                        double b[rowsB][colsB], double out[rowsA][colsB]){
    if(colsA != rowsB)
        return -1;

    double tempA[rowsA][colsA];
    double tempB[rowsB][colsB];
    matrixCopy(rowsA, colsA, a, tempA);
    matrixCopy(rowsB, colsB, b, tempB);
    double sum = 0;
    int i, j, k;
    for(i = 0; i < rowsA; i++){
        for(j = 0; j < colsB; j++){
            for(k = 0; k < colsA; k++){
                sum += tempA[i][k]*tempB[k][j];
            }
            out[i][j] = sum;
            sum = 0;
        }
    }
    return 1;
}

void matrixTranspose(int rows, int cols, double in[rows][cols], double out[rows][cols]){
    double temp[rows][cols];
    matrixCopy(rows, cols, in, temp);
    int i, j;
    for(i = 0; i < rows; i++){
        for(j = 0; j < cols; j++){
            out[j][i] = temp[i][j];
        }
    }
}

/* Inverts a 2-by-2 matrix */
int matrixInvert(int rows, int cols, double in[rows][cols], double out[rows][cols]){
    if(rows != 2 || cols != 2)
        return -1;

    double temp[rows][cols];
    matrixCopy(rows, cols, in, temp);

    double determinant = temp[0][0] * temp[1][1] - temp[0][1] * temp[1][0];
    if(determinant == 0) 
        return -1;
    
    out[0][0] = temp[1][1];
    out[0][1] = -temp[0][1];
    out[1][0] = -temp[1][0];
    out[1][1] = temp[0][0];
    matrixScale(rows, cols, out, 1.0 / determinant, out);
    return 1;
}

long double matrixDeterminant(int rows, int cols, double in[rows][cols]){
    if(rows != 2 || cols != 2)
        return -1;
    
    return in[0][0] * in[1][1] - in[0][1] * in[1][0];
}

/* Multiplies a 1-by-n matrix with a n-by-1 matrix, resulting in a scalar value */
double matrixMultiplyToScalar(int rowsA, int colsA, int rowsB, int colsB, double a[][colsA], double b[][colsB]){
    if(rowsA != 1 || colsB != 1 || colsA != rowsB)
        return -1;

    int i;
    double sum = 0;
    for(i = 0; i < colsA; i++)
        sum += a[0][i] * b[i][0];
    return sum;
}

