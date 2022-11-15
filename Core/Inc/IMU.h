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

/*IMU handle------------------------------------------------------------------------------------------------*/

typedef struct _imu
{
	i3g4250d 	*hgyroscope;
	lsm303agr 	*heCompass;
	float		pitch;
	float		yaw;
	float		roll;
}imu;

/*public function prototypes--------------------------------------------------------------------------------*/

void sendSensorDataString(imu *handle, uint8_t mode);
void sendSensorDataFloat(imu *handle);

void imu_calcRotation_complementaryFilter(imu *handle);

#ifdef __cplusplus
extern }
#endif

#endif /* SRC_IMU_H_ */
