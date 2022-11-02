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

/*IMU constants---------------------------------------------------------------------------------------------*/

#define ALPHA 0.01

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

void imu_calcRotation_complementaryFilter(imu *handle);

#ifdef __cplusplus
extern }
#endif

#endif /* SRC_IMU_H_ */
