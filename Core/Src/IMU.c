/*
 * IMU.c
 *
 *  Created on: 01.11.2022
 *      Author: Moritz
 */

#include "IMU.h"

/*public function definitions-------------------------------------------------------------------------------*/

/* function:		sendSensorDataString
 * description:		send x, y and z data of current mode as string via the USB Port
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void sendSensorDataString(imu *handle, uint8_t mode)
{
	char tempChar[40];
	for(int i=0; i<40; i++)
		tempChar[i] = 0;
	if(mode == 0)
		sprintf(&tempChar[0], "%f\t%f\t%f\n", handle->hgyroscope->x, handle->hgyroscope->y, handle->hgyroscope->z);
	else if(mode == 1)
		sprintf(&tempChar[0], "%f\t%f\t%f\n", handle->heCompass->x_A, handle->heCompass->y_A, handle->heCompass->z_A);
	else if(mode == 2)
		sprintf(&tempChar[0], "%f\t%f\t%f\n", handle->heCompass->x_M, handle->heCompass->y_M, handle->heCompass->z_M);
	else
		sprintf(&tempChar[0], "%f\t%f\t%f\n", handle->pitch, handle->roll, handle->yaw);
	CDC_Transmit_FS((uint8_t*)tempChar,40);
}

/* function:		sendSensorDataFloat
 * description:		send all imu-Sensor Data as float with USB
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void sendSensorDataFloat(imu *handle)
{
	float temp[9] = {handle->hgyroscope->x, handle->hgyroscope->y, handle->hgyroscope->z, handle->heCompass->x_A, handle->heCompass->y_A, handle->heCompass->z_A, handle->heCompass->x_M, handle->heCompass->y_M, handle->heCompass->z_M};
	CDC_Transmit_FS((uint8_t*)temp,36);
}

/* function:		imu_calcRotation_complementaryFilter
 * description:		calculate pitch and roll angle for accelerometer and gyroscope and fuse the data with a complementary filter
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void imu_calcRotation_complementaryFilter(imu *handle)
{
	handle->pitch = COMPLEMENTARY_ALPHA * asin(-handle->heCompass->y_A/sqrt(SQUARE(handle->heCompass->x_A) + SQUARE(handle->heCompass->y_A) + SQUARE(handle->heCompass->z_A))) * RADTODEG
			+ (1 - COMPLEMENTARY_ALPHA) * (handle->pitch - handle->hgyroscope->x / ODRGYRO);
	handle->roll = COMPLEMENTARY_ALPHA * atan2(-handle->heCompass->x_A, handle->heCompass->z_A) * RADTODEG
			+ (1 - COMPLEMENTARY_ALPHA) * (handle->roll - handle->hgyroscope->y / ODRGYRO);
}
