#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_base.h"
#include "ecrobot_private.h"
#include "ecrobot_interface.h"
#include "dist_nx.h"
#include "matrix_algebra.h"
#include "PID.h"
#include "weapon_system.h"
#include "math.h"

#define WSMOTOR1 NXT_PORT_B
#define WSMOTOR2 NXT_PORT_C


#define ANGLE 60
#define SAMPLESIZE 3

#define LEFT_4 -4
#define LEFT_3 -3
#define LEFT_2 -2
#define LEFT_1 -1
#define CENTER  0
#define RIGHT_1 1
#define RIGHT_2 2
#define RIGHT_3 3
#define RIGHT_4 4
#define UNKNOWN 5

#define RANGE_FAR 300
#define RANGE_CLOSE 700

#define MOTOR_FAST 80
#define MOTOR_MED  75
#define MOTOR_SLOW 70
#define MOTOR_STOP  0

#define LEFT_SENSOR NXT_PORT_S1
#define RIGHT_SENSOR NXT_PORT_S2

#define VK 8.0

#define off(X,Y) ((X/13.0)/(1.0 + pow(2.718, -0.01*((1/Y)-60))))

DeclareCounter(SysTimerCnt);
DeclareTask(BackgroundTask);
DeclareTask(KalmanTask);
DeclareTask(WeaponSystemTask);
DeclareTask(ShootingTask);

U32 WSRotation = 0;
S8 resetRotation = 0;
double kalmanReading = 0;
double x[2][1] = {{ -42.0}, {50.0}};
S8 targetSeenFlag = 0;
S8 prev = UNKNOWN;
S8 startCounter = 0;
S32 shootFlag = 0;
U8 enableKalmanTaskFlag = 0;
long double d = 11;
double P[2][2] = {{0.42, 0.0}, {0.0, 900.0}};
U8 resetCounter = 0;
U32 lastTimeStep = 0;
S32 offset = 0;
double K[2][1] = {{0.0}, {0.0}};

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void) {
    StatusType ercd;

    ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
    if (ercd != E_OK) {
        ShutdownOS(ercd);
    }
}


void reset() {
    kalmanReading = 0;
    x[0][0] = -42.0;
    x[1][0] = 50.0;
    targetSeenFlag = 0;
    prev = UNKNOWN;
    startCounter = 0;
    enableKalmanTaskFlag = 0;
    d = 11;
    shootFlag = 0;
    lastTimeStep = 0;
    offset = 0;
    P[0][0] = 0.42;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 900.0;
    nxt_motor_set_count(NXT_PORT_A, 0);
}

void resetTowerTo(S32 pos) {
    U32 start = systick_get_ms();
    while (start + 1500 > systick_get_ms()) {
        MotorPID(pos, NXT_PORT_A, 0);
    }
    nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    systick_wait_ms(500);
}

void ecrobot_device_initialize(void) {
    ecrobot_init_dist_sensor(NXT_PORT_S1, RANGE_MEDIUM, 1);
    ecrobot_init_dist_sensor(NXT_PORT_S2, RANGE_MEDIUM, 0);
}

void ecrobot_device_terminate(void) {
    ecrobot_term_dist_sensor(NXT_PORT_S1);
    ecrobot_term_dist_sensor(NXT_PORT_S2);
    nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    nxt_motor_set_speed(NXT_PORT_B, 0, 1);
    nxt_motor_set_speed(NXT_PORT_C, 0, 1);
}

void kalman(double zn) {
    /* Constants and matrices used for kalman calculations */
    static double R = VK;
    static double I[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
    U32 currentTimeStep = systick_get_ms();
    double t = (lastTimeStep == 0) ? 0.01 : (double)(currentTimeStep - lastTimeStep) / 1000.0;
    lastTimeStep = currentTimeStep;
    double h[1][2] = {{1.0, 0.0}};
    double hT[2][1] = {{1.0}, {0.0}};
    double a[2][2] = {{1.0, t}, {0.0, 1.0}};
    double aT[2][2] = {{1.0, 0.0}, {t, 1.0}};
    double y[2][1] = {{0.0}, {0.0}};
    double temp[1][2] = {{0.0, 0.0}};
    double temp2v = 0.0;
    double (*temp2)[1] = (double ( *)[1])(&temp2v);
    double temp3[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

    /* Project into timestep k+1 */
    matrixMultiply(2, 2, 2, 1, a, x, x);
    matrixMultiply(2, 2, 2, 2, a, P, P);
    matrixMultiply(2, 2, 2, 2, P, aT, P);
    d = matrixDeterminant(2, 2, P);

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

}

TASK(WeaponSystemTask) {
    MotorPID(WSRotation, WSMOTOR1, 0);
    MotorPID(WSRotation, WSMOTOR2, 0);
    TerminateTask();
}

TASK(KalmanTask) {
    if (enableKalmanTaskFlag) {

        S32 left = (S32)ecrobot_get_dist_sensor(LEFT_SENSOR);
        S32 right = (S32)ecrobot_get_dist_sensor(RIGHT_SENSOR);

        if (left < RANGE_CLOSE && right < RANGE_CLOSE)
            prev = CENTER;
        else if (left < RANGE_FAR)
            prev = LEFT_2;
        else if (left < RANGE_CLOSE) {
            if (prev == UNKNOWN || prev == LEFT_4 || prev == LEFT_3)
                prev = LEFT_3;
            else
                prev = LEFT_1;
        } else if (right < RANGE_FAR)
            prev = RIGHT_2;
        else if (right < RANGE_CLOSE) {
            if (prev == UNKNOWN || prev == RIGHT_4 || prev == RIGHT_3)
                prev = RIGHT_3;
            else
                prev = RIGHT_1;
        } else if (prev < 0)
            prev = LEFT_4;
        else if (prev > 0 && prev != UNKNOWN)
            prev = RIGHT_4;
        else {
            prev = UNKNOWN;
            startCounter = 0;
        }

        if (prev == LEFT_3 || prev == LEFT_2)
            kalmanReading = 17.37;
        else if (prev == RIGHT_3 || prev == RIGHT_2)
            kalmanReading = -20.74;
        else if (prev == LEFT_4)
            kalmanReading = 30.0;
        else if (prev == RIGHT_4)
            kalmanReading = -20.74;
        else if (prev == UNKNOWN)
            nxt_motor_set_speed(NXT_PORT_A, 0, 0);
        else if (prev == CENTER)
            kalmanReading = -1.058;
        else if (prev == LEFT_1)
            kalmanReading = 7.52;
        else
            kalmanReading = -9.78;

        if (startCounter < 10 && prev != UNKNOWN) {
            startCounter++;
            prev = UNKNOWN;
        }

        else if (prev != UNKNOWN) {
            kalmanReading += (double)(-nxt_motor_get_count(NXT_PORT_A));

            kalman(kalmanReading);
            S32 motor_pos = -nxt_motor_get_count(NXT_PORT_A);
            if ((motor_pos <= 100) && (motor_pos >= -45) ) {
                targetSeenFlag = 1;
                offset = (S32)off(x[1][0], d);
                offset = (offset == 0) ? 2 : offset;
                MotorPID((U32) (x[0][0] + offset), NXT_PORT_A, 1);
            } else
                nxt_motor_set_speed(NXT_PORT_A, 0, 1);
        }
    }
    TerminateTask();
}

int motor_in_range(int range) {
    return (-nxt_motor_get_count(NXT_PORT_A) > ((S32)(x[0][0]) + offset - range) &&
            -nxt_motor_get_count(NXT_PORT_A) < ((S32)(x[0][0]) + offset + range));
}

TASK(ShootingTask) {
    if (targetSeenFlag) {

        cock();
        if (-nxt_motor_get_count(NXT_PORT_A) > 30 && !shootFlag && motor_in_range(3)) {
            shootFlag = fire();
            resetCounter = 1;
        }
    }
    TerminateTask();
}

TASK(BackgroundTask) {
    resetTowerTo(35);
    reset();
    enableKalmanTaskFlag = 1;

    while (1) {
        if (resetCounter == 6) {
            enableKalmanTaskFlag = 0;
            targetSeenFlag = 0;
            WSRotation -= 150;
            resetTowerTo(0 - resetRotation);
            resetRotation -= -nxt_motor_get_count(NXT_PORT_A);
        }

        if (resetCounter == 12) {
            resetCounter = 0;
            reset();
            enableKalmanTaskFlag = 1;
        }

        if (resetCounter > 0) {
            resetCounter++;
        }

        display_clear(1);
        display_goto_xy(0, 0);
        display_int(WSRotation, 7);

        display_goto_xy(0, 1);
        display_int((U32)offset, 7);
        display_goto_xy(0, 2);
        display_int(nxt_motor_get_count(WSMOTOR2), 7);
        display_goto_xy(0, 3);
        display_int((int)(P[0][0] * 100), 7);
        display_goto_xy(0, 4);
        display_int((int)(P[1][1] * 100), 7);
        display_goto_xy(0, 5);
        display_int((int)(P[1][0] * 100), 7);
        display_goto_xy(0, 6);
        display_int((int)(P[0][1] * 100), 7);
        display_update();

        systick_wait_ms(100);
    }
}
