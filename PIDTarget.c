#include "ecrobot_interface.h"
#include "PIDTarget.h"
#include "dist_nx.h"

S8 directionCheck(S8 sensor1, S8 sensor2){

	double sensorLeft = ecrobot_get_dist_sensor(sensor1);
	double sensorRight = ecrobot_get_dist_sensor(sensor2);
//	static S32 lcbuf[SAMPLESIZE] = {{0}};
//	static S32 rcbuf[SAMPLESIZE] = {{0}};
//	static S32 i = 0;
//
//	lcbuf[i] = sensor1;
//	rcbuf[i] = sensor2;
//
//	i = (i == SAMPLESIZE-1) ? 0 : i+1;
//
//	S32 sensorLeft = 0;
//	S32 sensorRight = 0;
//
//	U8 j;
//	for(j = 0; j < SAMPLESIZE; j++){
//		sensorLeft += lcbuf[j];
//		sensorRight += rcbuf[j];
//	}
//
//	sensorLeft = sensorLeft / SAMPLESIZE;
//	sensorRight = sensorRight /SAMPLESIZE;

//	static S32 sensorLeft = 0;
//	static S32 sensorRight = 0;
//
//	static U8 lcount = 0;
//	static U8 rcount = 0;
//
//	if(sensorL < WALL){
//			sensorLeft = sensorL;
//			lcount = 2;
//		}
//		else if(lcount == 0)
//			sensorLeft = sensorL;
//		else
//			lcount = lcount-1;
//
//	if(sensorR < WALL){
//			sensorRight = sensorR;
//			rcount = 2;
//		}
//		else if(rcount == 0)
//			sensorRight = sensorR;
//		else
//			rcount = rcount-1;


	return (sensorLeft >= WALL && sensorRight >= WALL) ? TARGET_NO :
			(sensorLeft < WALL && sensorRight >= WALL) ? TARGET_LEFT :
			(sensorLeft >= WALL && sensorRight < WALL) ? TARGET_RIGHT : TARGET_CENTER;
}

S32 PIDTarget(S8 target)
{
	static S32 lastError = 0;
	static S32 integrale = 0;
	static S16 noTarget = 0;
	static S8 hadTarget = 0;

	if(target != TARGET_NO)
	{
		S32 derivative = target - lastError;
		integrale += target;

		double speed = target * Kp + integrale *Ki + derivative * Kd;
		S32 intSpeed = (int)speed;
		hadTarget = (intSpeed >> 31) | 1;

		return (speed > 0) ?
		            (S32)(speed + 50) :
		            (speed < 0) ?
		               (S32)(speed - 50) :
		                (S32)speed;
	}
	else
	{
		if(noTarget++ == 10 && hadTarget != 0)
		{
			lastError = 0;
			integrale = 0;
			noTarget = 0;
			hadTarget = 0;
		}

		return hadTarget * 75;
	}
}
