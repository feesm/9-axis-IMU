/*
 * IMU.h
 *
 *  Created on: 01.11.2022
 *      Author: Moritz
 */

#ifndef SRC_IMU_H_
#define SRC_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/*IMU includes----------------------------------------------------------------------------------------------*/

#include "i3g4250d.h"
#include "lsm303agr.h"
#include <math.h>
#include <stdio.h>


/*IMU macros----------------------------------------------------------------------------------------------*/

#ifndef SQUARE
#define SQUARE(BASE) (BASE * BASE)
#endif

/*IMU constants---------------------------------------------------------------------------------------------*/

#define COMPLEMENTARY_ALPHA	0.01
#define RADTODEG		57.295780181884765625
#define ODRGYRO			800

/*kalman state status---------------------------------------------------------------------------------------*/

typedef enum _kalman_state
{
	KAL_NONE,
	KAL_INIT,
	KAL_UPDATE,
	KAL_PREDICT
}kalman_state;

/*IMU handle------------------------------------------------------------------------------------------------*/

typedef struct _imu
{
	i3g4250d 	*hgyroscope;
	lsm303agr 	*heCompass;
	float		pitch;
	float		yaw;
	float		roll;
	float		p[9];
	float		t_last;
	float		mag_x;
	float		mag_z;
	kalman_state	state;
}imu;

/*public function prototypes--------------------------------------------------------------------------------*/

void sendSensorDataString(imu *handle, uint8_t mode);
void sendSensorDataFloat(imu *handle);

void imu_calcRotation_complementaryFilter(imu *handle);

void imu_init_kalmanFilter(imu *handle, i3g4250d *gyro, lsm303agr *eCompass);
void imu_predictAngles_kalmanFilter(imu *handle);
void imu_updateAngles_acc_EKF(imu *handle);
void imu_updateAngles_mag_EKF(imu *handle);

#ifdef __cplusplus
extern }
#endif

#endif /* SRC_IMU_H_ */
