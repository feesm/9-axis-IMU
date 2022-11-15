/*
 * lsm303agr.c
 *
 *  Created on: Oct 29, 2022
 *      Author: Moritz
 */
#include "lsm303agr.h"


/* private function prototypes--------------------------------------------------------------*/

static HAL_StatusTypeDef lsm303agr_readRegister(lsm303agr *handle, uint8_t address, uint8_t register, uint8_t *buf, uint8_t size);
static HAL_StatusTypeDef lsm303agr_writeRegister(lsm303agr *handle, uint8_t address, uint8_t register, uint8_t buf, uint8_t size);
static void lsm303agr_getMultiplicator(lsm303agr *handle);
static uint8_t lsm303agr_addToTaskQueue(lsm303agr *handle, lsm303agr_sensorTask task);
static HAL_StatusTypeDef lsm303agr_changeRange_A(lsm303agr *handle, uint8_t range);
static void lsm303agr_adjustRange_A(lsm303agr *handle);

/* private function definitions-------------------------------------------------------------*/


/* function:		lsm303agr_readRegister
 * description:		read lsm303agr register in blocking mode with I2C
 ***************************************************************
 * *handle:		pointer to sensor handle
 * address		I2C address of sensor
 * reg			sensor register
 * *rxBuf		pointer to reception array
 * size			size of data to be received in byte
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef lsm303agr_readRegister(lsm303agr *handle, uint8_t address, uint8_t reg, uint8_t *rxBuf, uint8_t size)
{
	return HAL_I2C_Mem_Read(handle->hi2c,address<<1,reg|(size==1?0x00:0x80),1,rxBuf,size,10);
}

/* function:		lsm303agr_writeRegister
 * description:		write lsm303agr register in blocking mode with I2C
 ***************************************************************
 * *handle:		pointer to sensor handle
 * address		I2C address of sensor
 * reg			sensor register
 * *txBuf		data to be send
 * size			size of data to be received in byte
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef lsm303agr_writeRegister(lsm303agr *handle, uint8_t address, uint8_t reg, uint8_t txBuf, uint8_t size)
{
	return HAL_I2C_Mem_Write(handle->hi2c,address<<1,reg|(size==1?0x00:0x80),1,&txBuf,size,10);
}

/* function:		lsm303agr_getMultiplicator
 * description:		get the multiplicator for acceleration values with precision and sensor range
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
static void lsm303agr_getMultiplicator(lsm303agr *handle)
{
	//high resolution mode
	if(handle->precision_A == 12
			&& handle->maxAbsValue_A == 2)
		handle->multiplicator_A = 0.00098;
	else if(handle->precision_A == 12
			&& handle->maxAbsValue_A == 4)
		handle->multiplicator_A = 0.00195;
	else if(handle->precision_A == 12
			&& handle->maxAbsValue_A == 8)
		handle->multiplicator_A = 0.0039;
	else if(handle->precision_A == 12
			&& handle->maxAbsValue_A == 16)
		handle->multiplicator_A = 0.01172;
	//normal mode
	else if(handle->precision_A == 10
			&& handle->maxAbsValue_A == 2)
		handle->multiplicator_A = 0.0039;
	else if(handle->precision_A == 10
			&& handle->maxAbsValue_A == 4)
		handle->multiplicator_A = 0.00782;
	else if(handle->precision_A == 10
			&& handle->maxAbsValue_A == 8)
		handle->multiplicator_A = 0.01563;
	else if(handle->precision_A == 10
			&& handle->maxAbsValue_A == 16)
		handle->multiplicator_A = 0.00469;
	//low-power mode
	else if(handle->precision_A == 8
			&& handle->maxAbsValue_A == 2)
		handle->multiplicator_A = 0.01563;
	else if(handle->precision_A == 8
			&& handle->maxAbsValue_A == 4)
		handle->multiplicator_A = 0.03126;
	else if(handle->precision_A == 8
			&& handle->maxAbsValue_A == 8)
		handle->multiplicator_A = 0.06252;
	else if(handle->precision_A == 8
			&& handle->maxAbsValue_A == 16)
		handle->multiplicator_A = 0.18758;

	else
		handle->multiplicator_A = 0.0;
}

/* function:		lsm303agr_addToTaskQueue
 * description:		add task to the end of the task queue
 ***************************************************************
 * *handle:		pointer to sensor handle
 * task:		task to add to the queue
 ***************************************************************
 * returns:		success of function
 */
static uint8_t lsm303agr_addToTaskQueue(lsm303agr *handle, lsm303agr_sensorTask task)
{
	for(int i = 0; i < LSM303AGR_TASKQUEUESIZE; i++)
		if(handle->nextTask[i] == lsm303agr_NONE)
		{
			handle->nextTask[i] = task;
			return SUCCESS;
		}
	return ERROR;
}

/* function:		lsm303agr_changeRange
 * description:		change lsm303agr full-scale
 ***************************************************************
 * *handle:		pointer to sensor handle
 * range:		new sensor acceleration range
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef lsm303agr_changeRange_A(lsm303agr *handle, uint8_t range)
{
	if(!LSM303AGR_READY(handle))
	{
		lsm303agr_sensorTask task;
		switch(range)
		{
			case 2:
			{
				task = lsm303agr_ChangeAccRange2g;
				break;
			}
			case 4:
			{
				task = lsm303agr_ChangeAccRange4g;
				break;
			}
			case 8:
			{
				task = lsm303agr_ChangeAccRange8g;
				break;
			}
			case 16:
			{
				task = lsm303agr_ChangeAccRange16g;
				break;
			}
			default:
			{
				return HAL_ERROR;
			}
		}
		if(lsm303agr_addToTaskQueue(handle, task) == SUCCESS)
			return HAL_BUSY;
		else
			return HAL_ERROR;
	}
	handle->currentTask = lsm303agr_ChangeAccRange;
	uint8_t fs = 0x00;
	switch(range)
	{
		case 2:
		{
			fs = 0x00;
			break;
		}
		case 4:
		{
			fs = 0x01;
			break;
		}
		case 8:
		{
			fs = 0x02;
			break;
		}
		case 16:
		{
			fs = 0x03;
			break;
		}
		default:
		{
			return HAL_ERROR;
		}
	}
	handle->maxAbsValue_A = range;
	lsm303agr_getMultiplicator(handle);
	handle->rxBuf[0] = CTRL_REG4_A;
	handle->rxBuf[1] = DEFAULT_CTRL_REG4_A | (fs<<4);
	return HAL_I2C_Master_Transmit_DMA(handle->hi2c, ACCELEROMETER<<1, handle->rxBuf, 2);
}

/* function:		lsm303agr_adjustRange
 * description:		adjust acceleration full scale to best value
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
static void lsm303agr_adjustRange_A(lsm303agr *handle)
{
	if(LSM303AGR_ACCINRANGE(handle, -1.9F, 1.9F))
	{
		if(handle->maxAbsValue_A != 2)
			lsm303agr_changeRange_A(handle, 2);
	}
	else if(LSM303AGR_ACCINRANGE(handle, -3.9F, 3.9F))
	{
		if(handle->maxAbsValue_A != 4)
			lsm303agr_changeRange_A(handle, 4);
	}
	else if(LSM303AGR_ACCINRANGE(handle, -7.9F, 7.9F))
	{
		if(handle->maxAbsValue_A != 8)
			lsm303agr_changeRange_A(handle, 8);
	}
	else
	{
		if(handle->maxAbsValue_A != 16)
			lsm303agr_changeRange_A(handle, 16);
	}
}


/*
 * public function definitions--------------------------------------------------------------
 */

/* function:		lsm303agr_config
 * description:	setup the lsm303agr sensor handle and configure the setup registers
 ***************************************************************
 * *handle:		pointer to sensor handle
 * *I2C_hi2c		pointer to SPI handle
 * I2C_hdma_rx		I2C rx to memory DMA handle
 * I2C_hdma_tx		memory to I2C tx DMA handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef lsm303agr_config(lsm303agr *handle, I2C_HandleTypeDef *I2C_hi2c, DMA_HandleTypeDef *I2C_hdma_rx, DMA_HandleTypeDef *I2C_hdma_tx)
{
	handle->currentTask = lsm303agr_Config;
	HAL_StatusTypeDef status = HAL_OK;
	//setup sensor related variables
	handle->hi2c = I2C_hi2c;
	handle->hdma_rx = I2C_hdma_rx;
	handle->hdma_tx = I2C_hdma_tx;
	handle->precision_A=12;
	handle->maxAbsValue_A=2;
	lsm303agr_getMultiplicator(handle);
	//write setup registers
	status = lsm303agr_writeRegister(handle, ACCELEROMETER, CTRL_REG3_A, DEFAULT_CTRL_REG3_A, 1);
	status = lsm303agr_writeRegister(handle, ACCELEROMETER, CTRL_REG1_A, DEFAULT_CTRL_REG1_A, 1);
	status = lsm303agr_writeRegister(handle, ACCELEROMETER, CTRL_REG4_A, DEFAULT_CTRL_REG4_A, 1);
	status = lsm303agr_writeRegister(handle, MAGNETICSENSOR, CFG_REG_A_M, DEFAULT_CFG_REG_A_M, 1);
	status = lsm303agr_writeRegister(handle, MAGNETICSENSOR, CFG_REG_B_M, DEFAULT_CFG_REG_B_M, 1);
	status = lsm303agr_writeRegister(handle, MAGNETICSENSOR, CFG_REG_C_M, DEFAULT_CFG_REG_C_M, 1);
	handle->currentTask = lsm303agr_NONE;
	for(int i = 0; i < 3; i++)
		handle->nextTask[i] = lsm303agr_NONE;
	return status;
}

/* function:		lsm303agr_readSensorData_A
 * description:		read lsm303agr acceleration raw data register in non blocking mode.
 * 			Raw Data will be saved in RxBuf array of handle.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef lsm303agr_readSensorData_A(lsm303agr *handle)
{
	if(!LSM303AGR_READY(handle))
	{
		if(lsm303agr_addToTaskQueue(handle, lsm303agr_GetAcceleration) == SUCCESS)
			return HAL_BUSY;
		else
			return HAL_ERROR;
	}
	handle->currentTask = lsm303agr_GetAcceleration;
	handle->txBuf[0]=OUT_X_L_A | 0x80;
	return HAL_I2C_Mem_Read_DMA(handle->hi2c,ACCELEROMETER<<1,handle->txBuf[0],1,&handle->rxBuf[0],6);
}

/* function:		lsm303agr_calcSensorData_A
 * description:		calculate the acceleration of lsm303agr with the raw values in the rxBuf array in the handle and low pass filter them.
 * 			lsm303agr_readSensorData_A have to be called before this function and the DMA have to been finished the SPI communication
 * 			to get correct values.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
void lsm303agr_calcSensorData_A(lsm303agr *handle)
{
	int16_t raw[3];
	for(int i=0;i<3;i++)
	{
		raw[i]=handle->rxBuf[2*i]|handle->rxBuf[2*i+1]<<8;
		raw[i]>>=16 - handle->precision_A;
	}
	handle->x_A = LPF_ACC_ALPHA * handle->x_A + (1 - LPF_ACC_ALPHA) * (raw[0]*handle->multiplicator_A - OFFSET_X_A);
	handle->y_A = LPF_ACC_ALPHA * handle->y_A + (1 - LPF_ACC_ALPHA) * (raw[1]*handle->multiplicator_A - OFFSET_Y_A);
	handle->z_A = LPF_ACC_ALPHA * handle->z_A + (1 - LPF_ACC_ALPHA) * (raw[2]*handle->multiplicator_A - OFFSET_Z_A);

	lsm303agr_adjustRange_A(handle);
}


/* function:		lsm303agr_readSensorData_M
 * description:		read lsm303agr magnetic raw data register in non blocking mode.
 * 			Raw Data will be saved in RxBuf array of handle.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef lsm303agr_readSensorData_M(lsm303agr *handle)
{
	if(!LSM303AGR_READY(handle))
	{
		if(lsm303agr_addToTaskQueue(handle, lsm303agr_GetMagneticFieldStrength) == SUCCESS)
			return HAL_BUSY;
		else
			return HAL_ERROR;
	}
	handle->currentTask = lsm303agr_GetMagneticFieldStrength;
	handle->txBuf[0]=OUTX_L_REG_M | 0x80;
	lsm303agr_addToTaskQueue(handle, lsm303agr_SetSingleMode);
	return HAL_I2C_Mem_Read_DMA(handle->hi2c,MAGNETICSENSOR<<1,handle->txBuf[0],1,&handle->rxBuf[0],6);
}

/* function:		lsm303agr_setSingleMode_M
 * description:		write lsm303agr's CFG_REG_A_M register to set the magnetic sensor to single mode
 * 			after new data is available,  lsm303agr be automatically set in idle mode
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef lsm303agr_setSingleMode_M(lsm303agr *handle)
{
	if(!LSM303AGR_READY(handle))
	{
		if(lsm303agr_addToTaskQueue(handle, lsm303agr_SetSingleMode) == SUCCESS)
			return HAL_BUSY;
		else
			return HAL_ERROR;
	}
	handle->currentTask = lsm303agr_SetSingleMode;
	handle->txBuf[0] = CFG_REG_A_M;
	handle->txBuf[1] = 0x81;
	return HAL_I2C_Master_Transmit_DMA(handle->hi2c,MAGNETICSENSOR<<1,&handle->txBuf[0],2);
}

/* function:		lsm303agr_calcSensorData_M
 * description:		calculate the magnetic field strength of lsm303agr with the raw values in the rxBuf array in the handle and low pass filter them.
 * 			lsm303agr_readSensorData_M have to be called before this function and the DMA have to been finished the SPI communication
 * 			to get correct values.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
void lsm303agr_calcSensorData_M(lsm303agr *handle)
{
	int16_t raw[3];
	for(int i=0;i<3;i++)
		raw[i]=handle->rxBuf[2*i]|handle->rxBuf[2*i+1]<<8;

	handle->x_M = LPF_MAG_ALPHA * handle->x_M + (1 - LPF_MAG_ALPHA) * (raw[0]*0.15);
	handle->y_M = LPF_MAG_ALPHA * handle->y_M + (1 - LPF_MAG_ALPHA) * (raw[1]*0.15);
	handle->z_M = LPF_MAG_ALPHA * handle->z_M + (1 - LPF_MAG_ALPHA) * (raw[2]*0.15);
}

/* function:		lsm303agr_startNextTask
 * description:		start next task from task queue
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
void lsm303agr_startNextTask(lsm303agr *handle)
{
	if(handle->nextTask[0] == lsm303agr_NONE)
		return;
	lsm303agr_sensorTask nextTask = handle->nextTask[0];
	for(int i = 0; i < LSM303AGR_TASKQUEUESIZE-1; i++)
		handle->nextTask[i] = handle->nextTask[i+1];
	handle->nextTask[LSM303AGR_TASKQUEUESIZE - 1] = lsm303agr_NONE;
	switch(nextTask)
	{
		case lsm303agr_GetAcceleration:
		{
			lsm303agr_readSensorData_A(handle);
			break;
		}
		case lsm303agr_GetMagneticFieldStrength:
		{

			lsm303agr_readSensorData_M(handle);
			break;
		}
		case lsm303agr_SetSingleMode:
		{
			lsm303agr_setSingleMode_M(handle);
			break;
		}
		case lsm303agr_ChangeAccRange2g:
		{
			lsm303agr_changeRange_A(handle, 2);
			break;
		}
		case lsm303agr_ChangeAccRange4g:
		{
			lsm303agr_changeRange_A(handle, 4);
			break;
		}
		case lsm303agr_ChangeAccRange8g:
		{
			lsm303agr_changeRange_A(handle, 8);
			break;
		}
		case lsm303agr_ChangeAccRange16g:
		{
			lsm303agr_changeRange_A(handle, 16);
			break;
		}
		default:
		{
			break;
		}
	}
}
