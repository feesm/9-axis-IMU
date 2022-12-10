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
		sprintf(&tempChar[0], "%f\t%f\n", handle->pitch, handle->roll);
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

/* function:		imu_init_kalmanFilter
 * description:		initialize values for kalman filter
 ***************************************************************
 * *handle:		pointer to imu handle
 * *gyro:		pointer to gyroscope handle
 * *eCompass:		pointer to eCompass handle
 ***************************************************************
 * returns:		-
 */

void imu_init_kalmanFilter(imu *handle, i3g4250d *gyro, lsm303agr *eCompass)
{
	  handle->heCompass = eCompass;
	  handle->hgyroscope = gyro;
	  handle->pitch = 0;
	  handle->roll = 0;
	  handle->yaw = 0;
	  for(int i=0; i<4;i++)
		  handle->p[i] = 0;
}

/* function:		imu_predictAngles_kalmanFilter
 * description:		predict state of kalman filter with euler angles. Should be called after new gyroscope values are available
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void imu_predictAngles_kalmanFilter(imu *handle)
{
	float p[4] = {handle->p[0],
		handle->p[1],
		handle->p[2],
		handle->p[3]};
	float q[4] = {0.001,0.001,0.001,0.001};

	/* get time t since last prediction **********************/
	float t = HAL_GetTick() / 1000.0F - handle->t_last;
	handle->t_last = HAL_GetTick() / 1000.0F;
	if(t > 4.0F/ODRGYRO)
		return;

	/* get angle with Euler integration of f(x) **************/
	handle->pitch -= (handle->hgyroscope->x * t)/ RADTODEG;
	handle->roll -= (handle->hgyroscope->y * t)/ RADTODEG;

	/* calculate jacobian A of function f(x)******************/
	float tp = tan(handle->pitch );
	float sr = sin(handle->roll );
	float cr = cos(handle->roll );

	float a[4];
	a[0] = tp * (handle->hgyroscope->x * cr - handle->hgyroscope->y * sr);
	a[1] = (tp * tp + 1.0F) * (handle->hgyroscope->x * sr + handle->hgyroscope->z * cr);
	a[2] = -handle->hgyroscope->x * sr - handle->hgyroscope->y * cr;
	a[3] = 0;

	/* update error convenience P of predicted angles ********/
	handle->p[0] = p[0] + t * (2.0F * a[0] *p[0] + a[1] * p[1] + a[1] * p[2] + q[0]);
	handle->p[1] = p[1] + t * (a[0] * p[1] + a[1] * p[3] + a[2] * p[0] + a[3] * p[1] + q[1]);
	handle->p[2] = p[2] + t * (a[0] * p[2] + a[1] * p[3] + a[2] * p[0] + a[3] * p[2] + q[2]);
	handle->p[3] = p[3] + t * (a[2] * p[1] + a[2] * p[2] + 2.0F * a[3] *p[3] + q[3]);
}

/* function:		imu_updateAngles_kalmanFilter
 * description:		update state of kalman filter with euler angles and apply corrections. Should be called after new accelerometer values are available
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void imu_updateAngles_kalmanFilter(imu *handle)
{
	float p[4] = {handle->p[0],	//temporary error convenience
		handle->p[1],
		handle->p[2],
		handle->p[3]
	};
	float r[9] = {0.011,0,0,0,0.011,0,0,0,0.011};
	float g = 9.81;

	float cr = cos(handle->roll );
	float cp = cos(handle->pitch );
	float sr = sin(handle->roll );
	float sp = sin(handle->pitch );

	/* predict values of accelerometer with estimated angles */
	float h[3];
	h[0] = g * sp;
	h[1] = -g * cp * sr;
	h[2] = -g * cp * cr;

	/* calculate jacobian C of function h(x) *****************/
	float c[6];
	c[0] = 0;
	c[1] = g * cp;
	c[2] = -g * cp * cr;
	c[3] = g * sr * sp;
	c[4] = g * sr * cp;
	c[5] = g * sp * cr;

	/* calculate kalman gain K *******************************/
	//G=c*p*ct+r
	float G[9];	//3x3 Matrix
	G[0] = c[1] * c[1] * p[3] + r[0];
	G[1] = c[2] * c[1] * p[2] + c[3] * c[1] * p[3] + r[1];
	G[2] = c[4] * c[1] * p[2] + c[5] * c[1] * p[3] + r[2];
	G[3] = c[1] * (c[2] * p[1] + c[3] * p[3]) + r[3];
	G[4] = c[2] * (c[2] * p[0] + c[3] * p[2]) + c[3] * (c[2] * p[1] + c[3] * p[3]) + r[4];
	G[5] = c[4] * (c[2] * p[0] + c[3] * p[2]) + c[5] * (c[2] * p[1] + c[3] * p[3]) + r[5];
	G[6] = c[1] * (c[4] * p[1] + c[5] * p[3]) + r[6];
	G[7] = c[2] * (c[4] * p[0] + c[5] * p[2]) + c[3] * (c[4] * p[1] + c[5] * p[3]) + r[7];
	G[8] = c[4] * (c[4] * p[0] + c[5] * p[2]) + c[5] * (c[4] * p[1] + c[5] * p[3]) + r[8];

	//numinv(G)
	float numinv[9];
	numinv[0] = G[4] * G[8] - G[5] * G[7];
	numinv[1] = G[2] * G[7] - G[1] * G[8];
	numinv[2] = G[1] * G[5] - G[2] * G[4];
	numinv[3] = G[5] * G[6] - G[3] * G[8];
	numinv[4] = G[0] * G[8] - G[2] * G[6];
	numinv[5] = G[2] * G[3] - G[0] * G[5];
	numinv[6] = G[3] * G[7] - G[4] * G[6];
	numinv[7] = G[1] * G[6] - G[0] * G[7];
	numinv[8] = G[0] * G[4] - G[1] * G[3];

	//inv(G)
	float inv[9];
	for (int i = 0; i < 9; i++)
		inv[i] = numinv[i] / (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8] + G[1] * G[5] * G[6] + G[2] * G[3] * G[7] - G[2] * G[4] * G[6]);

	//calculate kalman gain k=p*ct*y
	float k[6];
	k[0] = inv[0] * c[1] * p[1] + inv[3] * (c[2] * p[0] + c[3] * p[1]) + inv[6] * (c[4] * p[0] + c[5] * p[1]);
	k[1] = inv[1] * c[1] * p[1] + inv[4] * (c[2] * p[0] + c[3] * p[1]) + inv[7] * (c[4] * p[0] + c[5] * p[1]);
	k[2] = inv[2] * c[1] * p[1] + inv[5] * (c[2] * p[0] + c[3] * p[1]) + inv[8] * (c[4] * p[0] + c[5] * p[1]);
	k[3] = inv[0] * c[1] * p[3] + inv[3] * (c[2] * p[2] + c[3] * p[3]) + inv[6] * (c[4] * p[2] + c[5] * p[3]);
	k[4] = inv[1] * c[1] * p[3] + inv[4] * (c[2] * p[2] + c[3] * p[3]) + inv[7] * (c[4] * p[2] + c[5] * p[3]);
	k[5] = inv[2] * c[1] * p[3] + inv[5] * (c[2] * p[2] + c[3] * p[3]) + inv[8] * (c[4] * p[2] + c[5] * p[3]);

	/* apply correction to the estimated angles **************/
	handle->roll += + k[0] * (-handle->heCompass->y_A*g - h[0]) + k[1] * (handle->heCompass->x_A*g - h[1]) + k[2] * (-handle->heCompass->z_A*g - h[2]);
	handle->pitch +=  + k[3] * (-handle->heCompass->y_A*g - h[0]) + k[4] * (handle->heCompass->x_A*g - h[1]) + k[5] * (-handle->heCompass->z_A*g - h[2]);

	/* update error convenience P of predicted angles ********/
	handle->p[0] = p[0] * (- c[2] * k[1] - c[4] * k[2] + 1.0F) + p[2] * (-c[1] * k[0] - c[3] * k[1] - c[5] * k[2]);
	handle->p[1] = p[1] * (- c[2] * k[1] - c[4] * k[2] + 1.0F) + p[3] * (-c[1] * k[0] - c[3] * k[1] - c[5] * k[2]);
	handle->p[2] = p[0] * (- c[2] * k[4] - c[4] * k[5]) + p[2] * (-c[1] * k[3] - c[3] * k[4] - c[5] * k[5] + 1.0F);
	handle->p[3] = p[1] * (- c[2] * k[4] - c[4] * k[5]) + p[3] * (-c[1] * k[3] - c[3] * k[4] - c[5] * k[5] + 1.0F);
}
