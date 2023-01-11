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
	float temp[12] = {handle->hgyroscope->x, handle->hgyroscope->y, handle->hgyroscope->z, handle->heCompass->x_A, handle->heCompass->y_A, handle->heCompass->z_A, handle->heCompass->x_M, handle->heCompass->y_M, handle->heCompass->z_M, handle->pitch, handle->roll, handle->yaw};
	CDC_Transmit_FS((uint8_t*)temp,48);
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
	  for(int i=0; i<9;i++)
		  handle->p[i] = 0;
	  handle->mag_z = handle->heCompass->z_M;
	  handle->mag_x = sqrt(handle->heCompass->x_M * handle->heCompass->x_M + handle->heCompass->y_M * handle->heCompass->y_M);
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
	float p[9] = {handle->p[0],
			handle->p[1],
			handle->p[2],
			handle->p[3],
			handle->p[4],
			handle->p[5],
			handle->p[6],
			handle->p[7],
			handle->p[8]};

	/* get time t since last prediction **********************/
	float t = HAL_GetTick() / 1000.0F - handle->t_last;
	handle->t_last = HAL_GetTick() / 1000.0F;
	if(t > 4.0F/ODRGYRO)
		return;

	/* get the covariance of the sensor noise****************/
	float q[9] = {0.03 * sqrt(t), 0.0, 0.0,
			0.0, 0.03 * sqrt(t), 0.0,
			0.0, 0.0, 0.03 * sqrt(t)};

	/* get angle with Euler integration of f(x) **************/
	handle->pitch -= (handle->hgyroscope->x * t)/ RADTODEG;
	handle->roll -= (handle->hgyroscope->y * t)/ RADTODEG;
	handle->yaw -= (handle->hgyroscope->z * t)/ RADTODEG;

	/* calculate jacobian A of function f(x)******************/
	float sp = sin(handle->pitch );
	float cp = cos(handle->pitch );
	float tp = sp / cp;
	float sr = sin(handle->roll );
	float cr = cos(handle->roll );

	float a[9];
	a[1] = tp * (handle->hgyroscope->x * cr - handle->hgyroscope->y * sr);
	a[2] = (tp * tp + 1.0F) * (handle->hgyroscope->x * sr + handle->hgyroscope->z * cr);
	a[4] = -handle->hgyroscope->x * sr - handle->hgyroscope->y * cr;
	a[7] = (handle->hgyroscope->x * cr - handle->hgyroscope->y * sr) / cp;
	a[8] = (handle->hgyroscope->x * sr * sp + handle->hgyroscope->y * sp * cr) / (cp * cp);

	/* update error convenience P of predicted angles ********/
	handle->p[0] += t * (a[1] *p[1] + a[1] * p[3] + a[2] * p[2] + a[2] * p[6] + q[0]);
	handle->p[1] += t * (a[1] * p[4] + a[2] * p[7] + a[4] *p[1]);
	handle->p[2] += t * (a[1] * p[5] + a[2] * p[8] + a[7] *p[1] + a[8] * p[2]);
	handle->p[3] += t * (a[1] * p[4] + a[2] * p[5] + a[4] * p[3]);
	handle->p[4] += t * (2.0F * a[4] * p[4] + q[4]);
	handle->p[5] += t * (a[4] * p[5] + a[7] * p[4] + a[8] * p[5]);
	handle->p[6] += t * (a[1] * p[7] + a[2] * p[8] + a[7] * p[3] + a[8] * p[6]);
	handle->p[7] += t * (a[4] * p[7] + a[7] * p[4] + a[8] * p[7]);
	handle->p[8] += t * (a[7] * p[5] + a[7] * p[7] + 2.0F * a[8] * p[8] + q[8]);


}

/* function:		imu_updateAngles_kalmanFilter
 * description:		update state of kalman filter with euler angles and apply corrections. Should be called after new accelerometer values are available
 ***************************************************************
 * *handle:		pointer to imu handle
 ***************************************************************
 * returns:		-
 */
void imu_updateAngles_acc_EKF(imu *handle)
{
	float p[9] = {handle->p[0],
			handle->p[1],
			handle->p[2],
			handle->p[3],
			handle->p[4],
			handle->p[5],
			handle->p[6],
			handle->p[7],
			handle->p[8]};
	float r[9] = {0.01,0,0,0,0.01,0,0,0,0.01};
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
	float c[9];
	c[2] = g * cp;
	c[4] = -g * cp * cr;
	c[5] = g * sr * sp;
	c[7] = g * sr * cp;
	c[8] = g * sp * cr;

	/* calculate kalman gain K *******************************/
	//G=c*p*ct+r
	float G[9] = {c[2] * c[2] * p[8] + r[0],
			c[4] * c[2] * p[7] + c[5] * c[2] * p[8],
			c[7] * c[2] * p[7] + c[8] * c[2] * p[8],
			c[2] * (c[4] * p[5] + c[5] * p[8]),
			c[4] * (c[4] * p[4] + c[5] * p[7]) + c[5] * (c[4] * p[5] + c[5] * p[8]) + r[4],
			c[7] * (c[4] * p[4] + c[5] * p[7]) + c[8] * (c[4] * p[5] + c[5] * p[8]),
			c[2] * (c[7] * p[5] + c[8] * p[8]),
			c[4] * (c[7] * p[4] + c[8] * p[7]) + c[5] * (c[7] * p[5] + c[8] * p[8]),
			c[7] * (c[7] * p[4] + c[8] * p[7]) + c[8] * (c[7] * p[5] + c[8] * p[8]) + r[8]};

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
	float k[9] = {inv[0] * c[2] * p[2] + inv[3] * (c[4] * p[1] + c[5] * p[2]) + inv[6] * (c[7] * p[1] + c[8] * p[2]),
			inv[1] * c[2] * p[2] + inv[4] * (c[4] * p[1] + c[5] * p[2]) + inv[7] * (c[7] * p[1] + c[8] * p[2]),
			inv[2] * c[2] * p[2] + inv[5] * (c[4] * p[1] + c[5] * p[2]) + inv[8] * (c[7] * p[1] + c[8] * p[2]),
			inv[0] * c[2] * p[5] + inv[3] * (c[4] * p[4] + c[5] * p[5]) + inv[6] * (c[7] * p[4] + c[8] * p[5]),
			inv[1] * c[2] * p[5] + inv[4] * (c[4] * p[4] + c[5] * p[5]) + inv[7] * (c[7] * p[4] + c[8] * p[5]),
			inv[2] * c[2] * p[5] + inv[5] * (c[4] * p[4] + c[5] * p[5]) + inv[8] * (c[7] * p[4] + c[8] * p[5]),
			inv[0] * c[2] * p[8] + inv[3] * (c[4] * p[7] + c[5] * p[8]) + inv[6] * (c[7] * p[7] + c[8] * p[8]),
			inv[1] * c[2] * p[8] + inv[4] * (c[4] * p[7] + c[5] * p[8]) + inv[7] * (c[7] * p[7] + c[8] * p[8]),
			inv[2] * c[2] * p[8] + inv[5] * (c[4] * p[7] + c[5] * p[8]) + inv[8] * (c[7] * p[7] + c[8] * p[8])};

	/* apply correction to the estimated angles **************/
	handle->yaw += k[0] * (-handle->heCompass->y_A - h[0]) + k[1] * (handle->heCompass->x_A - h[1]) + k[2] * (-handle->heCompass->z_A - h[2]);
	handle->roll += k[3] * (-handle->heCompass->y_A - h[0]) + k[4] * (handle->heCompass->x_A - h[1]) + k[5] * (-handle->heCompass->z_A - h[2]);
	handle->pitch += k[6] * (-handle->heCompass->y_A - h[0]) + k[7] * (handle->heCompass->x_A - h[1]) + k[8] * (-handle->heCompass->z_A - h[2]);

	/* update error convenience P of predicted angles ********/
	handle->p[0] = p[0] + p[3] * (-c[4] * k[1] - c[7] * k[2]) + p[6] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[1] = p[1] + p[4] * (-c[4] * k[1] - c[7] * k[2]) + p[7] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[2] = p[2] + p[5] * (-c[4] * k[1] - c[7] * k[2]) + p[8] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[3] = p[3] * (-c[4] * k[4] - c[7] * k[5] + 1) + p[6] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[4] = p[4] * (-c[4] * k[4] - c[7] * k[5] + 1) + p[7] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[5] = p[5] * (-c[4] * k[4] - c[7] * k[5] + 1) + p[8] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[6] = p[3] * (-c[4] * k[7] - c[7] * k[8]) + p[6] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);
	handle->p[7] = p[4] * (-c[4] * k[7] - c[7] * k[8]) + p[7] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);
	handle->p[8] = p[5] * (-c[4] * k[7] - c[7] * k[8]) + p[8] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);
}
void imu_updateAngles_mag_EKF(imu *handle)
{
	//temporary error convenience
	float p[9] = {handle->p[0],
			handle->p[1],
			handle->p[2],
			handle->p[3],
			handle->p[4],
			handle->p[5],
			handle->p[6],
			handle->p[7],
			handle->p[8]};
	float r[9] = {3,0,0,0,3,0,0,0,3};

	volatile float sp = sin(handle->pitch);
	volatile float sr = sin(handle->roll);
	volatile float sy = sin(handle->yaw);
	volatile float cp = cos(handle->pitch);
	volatile float cr = cos(handle->roll);
	volatile float cy = cos(handle->yaw);

	/* predict values of magnetometer with estimated angles */
	float h[3] = {handle->mag_x * cy * cp - handle->mag_z * sp,
			handle->mag_x * (sr * sp * cy - sy * cr) + handle->mag_z * sr * cp,
			handle->mag_x * (sr * sy + sp * cr * cy) + handle->mag_z * cr * cp};

	/* calculate jacobian C of function h(x) *****************/
	float c[9] = {-handle->mag_x * sy * cp,
			0,
			-handle->mag_x * sp * cy - handle->mag_z * cp,
			handle->mag_x * (-sr * sy * sp - cr * cy),
			handle->mag_x * (-sr * sy - sp * cr * cy) + handle->mag_z * cr * cp,
			handle->mag_x * sr * cy * cp - handle->mag_z * sr * sp,
			handle->mag_x * (sr * cy - sy * sp * cr),
			handle->mag_x * (sr * sp * cy + sy * cr) - handle->mag_z * sr * cp,
			handle->mag_x * cr * cy * cp - handle->mag_z * sp * cr
			};

	/* calculate kalman gain K *******************************/
	//G=c*p*ct+r
	float G[9] = {c[0] *(c[0] * p[0] + c[1] * p[3] + c[2] * p[6]) + c[1] * (c[0] * p[1] + c[1] * p[4] + c[2] * p[7]) + c[2] * (c[0] * p[2] + c[1] * p[5] + c[2] * p[8]) + r[0],
			c[3] *(c[0] * p[0] + c[1] * p[3] + c[2] * p[6]) + c[4] * (c[0] * p[1] + c[1] * p[4] + c[2] * p[7]) + c[5] * (c[0] * p[2] + c[1] * p[5] + c[2] * p[8]) + r[1],
			c[6] *(c[0] * p[0] + c[1] * p[3] + c[2] * p[6]) + c[7] * (c[0] * p[1] + c[1] * p[4] + c[2] * p[7]) + c[8] * (c[0] * p[2] + c[1] * p[5] + c[2] * p[8]) + r[2],
			c[0] *(c[3] * p[0] + c[4] * p[3] + c[5] * p[6]) + c[1] * (c[3] * p[1] + c[4] * p[4] + c[5] * p[7]) + c[2] * (c[3] * p[2] + c[4] * p[5] + c[5] * p[8]) + r[3],
			c[3] *(c[3] * p[0] + c[4] * p[3] + c[5] * p[6]) + c[4] * (c[3] * p[1] + c[4] * p[4] + c[5] * p[7]) + c[5] * (c[3] * p[2] + c[4] * p[5] + c[5] * p[8]) + r[4],
			c[6] *(c[3] * p[0] + c[4] * p[3] + c[5] * p[6]) + c[7] * (c[3] * p[1] + c[4] * p[4] + c[5] * p[7]) + c[8] * (c[3] * p[2] + c[4] * p[5] + c[5] * p[8]) + r[5],
			c[0] *(c[6] * p[0] + c[7] * p[3] + c[8] * p[6]) + c[1] * (c[6] * p[1] + c[7] * p[4] + c[8] * p[7]) + c[2] * (c[6] * p[2] + c[7] * p[5] + c[8] * p[8]) + r[6],
			c[3] *(c[6] * p[0] + c[7] * p[3] + c[8] * p[6]) + c[4] * (c[6] * p[1] + c[7] * p[4] + c[8] * p[7]) + c[5] * (c[6] * p[2] + c[7] * p[5] + c[8] * p[8]) + r[7],
			c[6] *(c[6] * p[0] + c[7] * p[3] + c[8] * p[6]) + c[7] * (c[6] * p[1] + c[7] * p[4] + c[8] * p[7]) + c[8] * (c[6] * p[2] + c[7] * p[5] + c[8] * p[8]) + r[8]};

	//numinv(G)
	float numinv[9] = {G[4] * G[8] - G[5] * G[7],
			G[2] * G[7] - G[1] * G[8],
			G[1] * G[5] - G[2] * G[4],
			G[5] * G[6] - G[3] * G[8],
			G[0] * G[8] - G[2] * G[6],
			G[2] * G[3] - G[0] * G[5],
			G[3] * G[7] - G[4] * G[6],
			G[1] * G[6] - G[0] * G[7],
			G[0] * G[4] - G[1] * G[3]};

	//inv(G)
	float inv[9];
	for (int i = 0; i < 9; i++)
		inv[i] = numinv[i] / (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8] + G[1] * G[5] * G[6] + G[2] * G[3] * G[7] - G[2] * G[4] * G[6]);

	//calculate kalman gain k=p*ct*y
	float k[9] = {inv[0] * (c[0] * p[0] + c[1] * p[1] + c[2] * p[2]) + inv[3] * (c[3] * p[0] + c[4] * p[1] + c[5] * p[2]) + inv[6] * (c[6] * p[0] + c[7] * p[1] + c[8] * p[2]),
			inv[1] * (c[0] * p[0] + c[1] * p[1] + c[2] * p[2]) + inv[4] * (c[3] * p[0] + c[4] * p[1] + c[5] * p[2]) + inv[7] * (c[6] * p[0] + c[7] * p[1] + c[8] * p[2]),
			inv[2] * (c[0] * p[0] + c[1] * p[1] + c[2] * p[2]) + inv[5] * (c[3] * p[0] + c[4] * p[1] + c[5] * p[2]) + inv[8] * (c[6] * p[0] + c[7] * p[1] + c[8] * p[2]),
			inv[0] * (c[0] * p[3] + c[1] * p[4] + c[2] * p[5]) + inv[3] * (c[3] * p[3] + c[4] * p[4] + c[5] * p[5]) + inv[6] * (c[6] * p[3] + c[7] * p[4] + c[8] * p[5]),
			inv[1] * (c[0] * p[3] + c[1] * p[4] + c[2] * p[5]) + inv[4] * (c[3] * p[3] + c[4] * p[4] + c[5] * p[5]) + inv[7] * (c[6] * p[3] + c[7] * p[4] + c[8] * p[5]),
			inv[2] * (c[0] * p[3] + c[1] * p[4] + c[2] * p[5]) + inv[5] * (c[3] * p[3] + c[4] * p[4] + c[5] * p[5]) + inv[8] * (c[6] * p[3] + c[7] * p[4] + c[8] * p[5]),
			inv[0] * (c[0] * p[6] + c[1] * p[7] + c[2] * p[8]) + inv[3] * (c[3] * p[6] + c[4] * p[7] + c[5] * p[8]) + inv[6] * (c[6] * p[6] + c[7] * p[7] + c[8] * p[8]),
			inv[1] * (c[0] * p[6] + c[1] * p[7] + c[2] * p[8]) + inv[4] * (c[3] * p[6] + c[4] * p[7] + c[5] * p[8]) + inv[7] * (c[6] * p[6] + c[7] * p[7] + c[8] * p[8]),
			inv[2] * (c[0] * p[6] + c[1] * p[7] + c[2] * p[8]) + inv[5] * (c[3] * p[6] + c[4] * p[7] + c[5] * p[8]) + inv[8] * (c[6] * p[6] + c[7] * p[7] + c[8] * p[8])};

	/* apply correction to the estimated angles **************/
	handle->yaw += k[0] * (handle->heCompass->y_M - h[0]) + k[1] * (-handle->heCompass->x_M - h[1]) + k[2] * (handle->heCompass->z_M - h[2]);
	handle->roll += k[3] * (-handle->heCompass->y_M - h[0]) + k[4] * (-handle->heCompass->x_M - h[1]) + k[5] * (handle->heCompass->z_M - h[2]);
	handle->pitch += k[6] * (handle->heCompass->y_M - h[0]) + k[7] * (-handle->heCompass->x_M - h[1]) + k[8] * (handle->heCompass->z_M - h[2]);

	/* update error convenience P of predicted angles ********/
	handle->p[0] = p[0] * (-c[0] * k[0] - c[3] * k[1] - c[6] * k[2] + 1) + p[3] * (-c[1] * k[0] - c[4] * k[1] - c[7] * k[2]) + p[6] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[1] = p[1] * (-c[0] * k[0] - c[3] * k[1] - c[6] * k[2] + 1) + p[4] * (-c[1] * k[0] - c[4] * k[1] - c[7] * k[2]) + p[7] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[2] = p[2] * (-c[0] * k[0] - c[3] * k[1] - c[6] * k[2] + 1) + p[5] * (-c[1] * k[0] - c[4] * k[1] - c[7] * k[2]) + p[8] * (-c[2] * k[0] - c[5] * k[1] - c[8] * k[2]);
	handle->p[3] = p[0] * (-c[0] * k[3] - c[3] * k[4] - c[6] * k[5]) + p[3] * (-c[1] * k[3] - c[4] * k[4] - c[7] * k[5] + 1) + p[6] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[4] = p[1] * (-c[0] * k[3] - c[3] * k[4] - c[6] * k[5]) + p[4] * (-c[1] * k[3] - c[4] * k[4] - c[7] * k[5] + 1) + p[7] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[5] = p[2] * (-c[0] * k[3] - c[3] * k[4] - c[6] * k[5]) + p[5] * (-c[1] * k[3] - c[4] * k[4] - c[7] * k[5] + 1) + p[8] * (-c[2] * k[3] - c[5] * k[4] - c[8] * k[5]);
	handle->p[6] = p[0] * (-c[0] * k[6] - c[3] * k[7] - c[6] * k[8]) + p[3] * (-c[1] * k[6] - c[4] * k[7] - c[7] * k[8]) + p[6] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);
	handle->p[7] = p[1] * (-c[0] * k[6] - c[3] * k[7] - c[6] * k[8]) + p[4] * (-c[1] * k[6] - c[4] * k[7] - c[7] * k[8]) + p[7] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);
	handle->p[8] = p[2] * (-c[0] * k[6] - c[3] * k[7] - c[6] * k[8]) + p[5] * (-c[1] * k[6] - c[4] * k[7] - c[7] * k[8]) + p[8] * (-c[2] * k[6] - c[5] * k[7] - c[8] * k[8] + 1);

	/*keep yaw values between +pi and -pi*/
	if(handle->yaw > 3.141)
		handle->yaw -= 6.283;
	else if(handle->yaw < -3.141)
		handle->yaw += 6.283;
}
