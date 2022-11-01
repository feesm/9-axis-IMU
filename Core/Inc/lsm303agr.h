/*
 * lsm303agr.h
 *
 *  Created on: Oct 29, 2022
 *      Author: Moritz
 */

#ifndef INC_LSM303AGR_H_
#define INC_LSM303AGR_H_

#ifdef __cplusplus
extern "C" {
#endif

/*lsm303agr includes----------------------------------------------------------------------------------------*/

#include "stm32f3xx_hal.h"

/*lsm303agr macros------------------------------------------------------------------------------------------*/

#define LSM303AGR_READY(phandle)  (phandle->currentTask==lsm303agr_NONE \
		&& phandle->hi2c->State == HAL_I2C_STATE_READY \
		&& phandle->hdma_tx->State == HAL_DMA_STATE_READY \
		&& phandle->hdma_rx->State == HAL_DMA_STATE_READY)
#define INRANGE(value, minValue, maxValue) (value <= maxValue \
		&& value >= minValue)
#define LSM303AGR_ACCINRANGE(phandle, minValue, maxValue) (INRANGE(phandle->x_A, minValue, maxValue) \
		&& INRANGE(phandle->y_A, minValue, maxValue) \
		&& INRANGE(phandle->z_A, minValue, maxValue)) \

/*lsm303agr I2C addresses-----------------------------------------------------------------------------------*/

#define	ACCELEROMETER 		0x19
#define MAGNETICSENSOR  	0x1E

/*lsm303agr registers---------------------------------------------------------------------------------------*/

/*accelerometer registers*/

#define STATUS_REG_AUX_A	0x07
#define OUT_TEMP_L_A		0x0C
#define OUT_TEMP_H_A		0x0D
#define INT_COUNTER_REG_A	0x0E
#define WHO_AM_I_A		0x0F
#define TEMP_CFG_REG_A		0x1F
#define CTRL_REG1_A		0x20
#define CTRL_REG2_A		0x21
#define CTRL_REG3_A		0x22
#define CTRL_REG4_A		0x23
#define CTRL_REG5_A		0x24
#define CTRL_REG6_A		0x25
#define REFERENCE_A		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A		0x28
#define OUT_X_H_A		0x29
#define OUT_Y_L_A		0x2A
#define OUT_Y_H_A		0x2B
#define OUT_Z_L_A		0x2C
#define OUT_Z_H_A		0x2D
#define FIFO_CTRL_REG_A		0x2E
#define FIFO_SRC_REG_A		0x2F
#define INT1_CFG_A		0x30
#define INT1_SRC_A		0x31
#define INT1_THS_A		0x32
#define INT1_DURATION_A		0x33
#define INT2_CFG_A		0x34
#define INT2_SRC_A		0x35
#define INT2_THS_A		0x36
#define INT2_DURATION_A		0x37
#define CLICK_CFG_A		0x38
#define CLICK_SRC_A		0x39
#define CLICK_THS_A		0x3A
#define TIME_LIMIT_A		0x3B
#define TIME_LATENCY_A		0x3C
#define TIME_WINDOW_A		0x3D
#define Act_THS_A		0x3E
#define Act_DUR_A		0x3F

/*magnetic sensor registers*/

#define OFFSET_X_REG_L_M	0x45
#define OFFSET_X_REG_H_M	0x46
#define OFFSET_Y_REG_L_M	0x47
#define OFFSET_Y_REG_H_M	0x48
#define OFFSET_Z_REG_L_M	0x49
#define OFFSET_Z_REG_H_M	0x4A
#define WHO_AM_I_M		0x4F
#define CFG_REG_A_M		0x60
#define CFG_REG_B_M		0x61
#define CFG_REG_C_M		0x62
#define INT_CTRL_REG_M		0x63
#define INT_SOURCE_REG_M	0x64
#define INT_THS_L_REG_M		0x65
#define INT_THS_H_REG_M		0x66
#define STATUS_REG_M		0x67
#define OUTX_L_REG_M		0x68
#define OUTX_H_REG_M		0x69
#define OUTY_L_REG_M		0x6A
#define OUTY_H_REG_M		0x6B
#define OUTZ_L_REG_M		0x6C
#define OUTZ_H_REG_M		0x6D

/*lsm303agr constants---------------------------------------------------------------------------------------*/

#define DEFAULT_CTRL_REG1_A 	0x77
#define DEFAULT_CTRL_REG3_A 	0x10
#define DEFAULT_CTRL_REG4_A 	0x08
#define DEFAULT_CFG_REG_A_M 	0x81
#define DEFAULT_CFG_REG_B_M 	0x01
#define DEFAULT_CFG_REG_C_M 	0x01

#define LSM303AGR_TASKQUEUESIZE 3

/*lsm303agr typedefs----------------------------------------------------------------------------------------*/

typedef enum _lsm303agr_sensorTask
{
	lsm303agr_NONE=0x00,
	lsm303agr_GetAcceleration = 0x10,
	lsm303agr_GetMagneticFieldStrength = 0x11,
	lsm303agr_Config = 0x20,
	lsm303agr_SetSingleMode = 0x21,
	lsm303agr_ChangeAccRange = 0x30,
	lsm303agr_ChangeAccRange2g = 0x31,
	lsm303agr_ChangeAccRange4g = 0x32,
	lsm303agr_ChangeAccRange8g = 0x33,
	lsm303agr_ChangeAccRange16g = 0x34
}lsm303agr_sensorTask;

/*lsm303agr sensor handle*/

typedef struct _lsm303agr
{
	float x_A;
	float y_A;
	float z_A;
	float x_M;
	float y_M;
	float z_M;
	I2C_HandleTypeDef *hi2c;
	DMA_HandleTypeDef *hdma_tx;
	DMA_HandleTypeDef *hdma_rx;
	uint8_t rxBuf[6];
	uint8_t txBuf[2];
	uint8_t precision_A;
	float multiplicator_A;
	uint8_t maxAbsValue_A;
	lsm303agr_sensorTask currentTask;
	lsm303agr_sensorTask nextTask[LSM303AGR_TASKQUEUESIZE];
}lsm303agr;


/*public function prototypes--------------------------------------------------------------------------------*/


HAL_StatusTypeDef lsm303agr_config(lsm303agr *handle, I2C_HandleTypeDef *I2C_i2c, DMA_HandleTypeDef *I2C_hdma_rx, DMA_HandleTypeDef *I2C_hdma_tx);
HAL_StatusTypeDef lsm303agr_readSensorData_A(lsm303agr *handle);
void lsm303agr_calcSensorData_A(lsm303agr *handle);
HAL_StatusTypeDef lsm303agr_readSensorData_M(lsm303agr *handle);
HAL_StatusTypeDef lsm303agr_setSingleMode_M(lsm303agr *handle);
void lsm303agr_calcSensorData_M(lsm303agr *handle);
void lsm303agr_startNextTask(lsm303agr *handle);


#ifdef __cplusplus
extern }
#endif

#endif /* INC_LSM303AGR_H_ */
