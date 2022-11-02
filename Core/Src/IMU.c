/*
 * IMU.c
 *
 *  Created on: 01.11.2022
 *      Author: Moritz
 */

#include "IMU.h"


void imu_calcRotation_complementaryFilter(imu *handle)
{
	handle->pitch = ALPHA * atan(-handle->heCompass->y_A / sqrt(handle->heCompass->x_A * handle->heCompass->x_A + handle->heCompass->z_A * handle->heCompass->z_A)) * 57.29578 + (1 - ALPHA) * (handle->pitch + handle->hgyroscope->x / 800.0F);
	handle->roll  = ALPHA * atan(-handle->heCompass->x_A / sqrt(handle->heCompass->y_A * handle->heCompass->y_A + handle->heCompass->z_A * handle->heCompass->z_A)) * 57.29578 + (1 - ALPHA) * (handle->roll + handle->hgyroscope->y / 800.0F);
}
