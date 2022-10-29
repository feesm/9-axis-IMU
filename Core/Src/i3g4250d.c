/*
 * i3g4250d.c
 *
 *  Created on: Oct 20, 2022
 *      Author: Moritz
 */
#include "i3g4250d.h"

/*private function prototypes--------------------------------------------------------------------------------*/

static HAL_StatusTypeDef i3g4250d_readRegister(i3g4250d *handle, uint8_t reg, uint8_t *rxBuf, uint16_t size);
static HAL_StatusTypeDef i3g4250d_writeRegister(i3g4250d *handle,uint8_t reg,uint8_t txBuf);
static HAL_StatusTypeDef i3g4250d_writeRegisterDMA(i3g4250d *handle,uint8_t reg,uint8_t txBuf);

/*private function definitions-------------------------------------------------------------------------------*/


/* function:		i3g4250d_readRegister
 * description:		read i3g4250d register in blocking mode
 ***************************************************************
 * *handle:		pointer to sensor handle
 * reg:			address of register in sensor
 * *rxBuf:		Pointer to reception array
 * size:		size of data to be received in byte
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef i3g4250d_readRegister(i3g4250d *handle, uint8_t reg, uint8_t *rxBuf, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_OK;

	//check sensor state
	if(handle->status!=FINISH)
		return HAL_BUSY;

	handle->status=TRANSMITTING;

	handle->txBuf[0] = reg|(size==1?0x80:0xC0);


	HAL_GPIO_WritePin(handle->cs_Reg, handle->cs_Pin, GPIO_PIN_RESET);
	status=HAL_SPI_TransmitReceive(handle->hspi, &handle->txBuf[0], &handle->rxBuf[0], size+1, 5);

	for(int i=0; i<size; i++)
		rxBuf[i] = handle->rxBuf[i+1];

	if(status != HAL_BUSY)
	{
		handle->status = FINISH;
		HAL_GPIO_WritePin(handle->cs_Reg, handle->cs_Pin, GPIO_PIN_SET);
	}
	return status;
}

/* function:		i3g4250d_writeRegister
 * description:		write i3g4250d register in blocking mode
 ***************************************************************
 * *handle:		pointer to sensor handle
 * reg:			address of register in sensor
 * txBuf:		data to be send
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef i3g4250d_writeRegister(i3g4250d *handle,uint8_t reg,uint8_t txBuf)
{
	HAL_StatusTypeDef status = HAL_OK;

	//check sensor state
	if(handle->status!=FINISH)
		return HAL_BUSY;

	handle->status = TRANSMITTING;

	handle->txBuf[0] = reg;
	handle->txBuf[1] = txBuf;

	HAL_GPIO_WritePin(handle->cs_Reg, handle->cs_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(handle->hspi, &handle->txBuf[0], 2, 10);

	if(status != HAL_BUSY)
	{
		handle->status = FINISH;
		HAL_GPIO_WritePin(handle->cs_Reg, handle->cs_Pin, GPIO_PIN_SET);
	}
	return status;
}

/* function:		i3g4250d_writeRegisterDMA
 * description:		write i3g4250d register in non blocking mode
 ***************************************************************
 * *handle:		pointer to sensor handle
 * reg:			address of register in sensor
 * txBuf:		data to be send
 ***************************************************************
 * returns:		state of transfer
 */
static HAL_StatusTypeDef i3g4250d_writeRegisterDMA(i3g4250d *handle,uint8_t reg,uint8_t txBuf)
{
	//check sensor state
	if(handle->status!=FINISH)
		return HAL_BUSY;

	handle->status=TRANSMITTING;
	//setup SPI registers
	handle->txBuf[0]=reg;
	handle->txBuf[1]=txBuf;
	//start SPI communication
	HAL_GPIO_WritePin(GPIOE,handle->cs_Pin,GPIO_PIN_RESET);
	return HAL_SPI_Transmit_DMA(handle->hspi,&handle->txBuf[0],2);
}

/*public function definitions---------------------------------------------------------------------------------*/

/* function:		i3g4250d_config
 * description:		setup the i3g4250d sensor handle and configure the setup registers
 ***************************************************************
 * *handle:		pointer to sensor handle
 * *SPI_hspi		pointer to SPI handle
 * SPI_cs_Pin		SPI cs Pin number
 * SPI_cs_Reg		SPI cs register
 * SPI_hdma_rx		SPI rx to memory DMA handle
 * SPI_hdma_tx		memory to SPI tx DMA handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef i3g4250d_config(i3g4250d *handle, SPI_HandleTypeDef *SPI_hspi, uint16_t SPI_cs_Pin, GPIO_TypeDef * SPI_cs_Reg, DMA_HandleTypeDef *SPI_hdma_rx, DMA_HandleTypeDef *SPI_hdma_tx)
{
	HAL_StatusTypeDef status;
	//setup sensor related variables
	handle->hspi = SPI_hspi;
	handle->hdma_rx = SPI_hdma_rx;
	handle->hdma_tx = SPI_hdma_tx;
	handle->cs_Pin = SPI_cs_Pin;
	handle->cs_Reg = SPI_cs_Reg;
	for(int i=0;i<7;i++)
	{
		handle->rxBuf[i] = 0x00;
		handle->txBuf[i] = 0x00;
	}
	handle->temperature = 0x00;
	handle->status = FINISH;
	handle->next = FINISH;

	//write setup-registers
	uint8_t rxBuf = 0x00;
	status = i3g4250d_readRegister(handle,CTRL_REG4,&rxBuf,1);
	if(status != HAL_OK)
		return status;
	status = i3g4250d_writeRegister(handle,CTRL_REG1,0xFF);
	if(status != HAL_OK)
		return status;
	status = i3g4250d_writeRegister(handle,CTRL_REG3,0x08);
	if(status != HAL_OK)
		return status;

	//get measurement range
	switch((rxBuf&0x30)>>4)
	{
		case 0x00:
		{
			handle->measureMode = 245;
			break;
		}
		case 0x01:
		{
			handle->measureMode = 500;
			break;
		}
		case 0x02:
		{
			handle->measureMode = 2000;
			break;
		}
		case 0x03:
		{
			handle->measureMode = 2000;
			break;
		}
		default:
		{
			break;
		}
	}
	return status;
}

/* function:		i3g4250d_checkBlockedTask
 * description:		check if a task related to i3g4250d was blocked, because of a busy port and call that.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
void i3g4250d_checkBlockedTask(i3g4250d *handle)
{
	if(handle->next==FINISH)
		return;
	switch(handle->next)
	{
	case READSENSORDATA:
		{
			handle->next=FINISH;
			i3g4250d_readSensorData(handle);
			break;
		}
	case READTEMPERATURE:
		{
			handle->next=FINISH;
			i3g4250d_readTemperature(handle);
			break;
		}
	case ADJUSTRANGE:
		{
			handle->next=FINISH;
			i3g4250d_adjustRange(handle);
			break;
		}
	default:
		{
			handle->next=FINISH;
			break;
		}
	}
}

/* function:		i3g4250d_readSensorData
 * description:		read i3g4250d angular velocity raw data register in non blocking mode.
 * 			Raw Data will be saved in RxBuf array of handle.
 * 			SPI communication need to be finished with a high cs Pin and the i3g4250d-handle state need to be reset to finish.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef i3g4250d_readSensorData(i3g4250d *handle)
{
	//check if SPI, DMA or sensor is busy
	if(handle->hdma_tx->State!=0x01||handle->hdma_tx->State!=0x01||handle->hspi->State!=0x01||handle->status!=FINISH)
	{
		handle->next=READSENSORDATA;
		return HAL_BUSY;
	}
	handle->txBuf[0]=OUT_X_L|0xC0;
	for(int i=0x01;i<7;i++)
		handle->txBuf[i]=0x00;
	handle->status=READSENSORDATA;
	//setup DMA for Reading SensorData
	HAL_GPIO_WritePin(GPIOE,handle->cs_Pin,GPIO_PIN_RESET);
	return HAL_SPI_TransmitReceive_DMA(handle->hspi,&handle->txBuf[0],&handle->rxBuf[0],6+1);
}

/* function:		i3g4250d_calcSensorData
 * description:		calculate the angular velocity of i3g4250d with the raw values in the rxBuf array in the handle.
 * 			i3g4250d_readSensorData have to be called before this function and the DMA have to been finished the SPI communication
 * 			to get correct values.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		-
 */
void i3g4250d_calcSensorData(i3g4250d *handle)
{

	//calculate data
	float fac=0;
	if(handle->measureMode==245)
		fac=0.00875;
	else if(handle->measureMode==500)
		fac=0.0175;
	else if(handle->measureMode==2000)
		fac=0.07;
	int16_t raw[3]={0,0,0};
	raw[0]=(handle->rxBuf[1]|(handle->rxBuf[2]<<8));
	raw[1]=(handle->rxBuf[3]|(handle->rxBuf[4]<<8));
	raw[2]=(handle->rxBuf[5]|(handle->rxBuf[6]<<8));
	handle->x=raw[0]*fac;
	handle->y=raw[1]*fac;
	handle->z=raw[2]*fac;
	handle->status=FINISH;
}

/* function:		i3g4250d_adjustRange
 * description:		adjusts the sensor range to the smallest value where all sensor values are included
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef i3g4250d_adjustRange(i3g4250d *handle)
{
	//check if SPI or DMA is busy
	if(handle->hdma_tx->State!=0x01||handle->hdma_tx->State!=0x01||handle->hspi->State!=0x01||handle->status!=FINISH)
	{
		handle->next=ADJUSTRANGE;
		return HAL_BUSY;
	}
	//check if all values are included in the smallest sensor range
	if(handle->x<245	&&	handle->y<245	&&	handle->z<245)
	{
		if(handle->measureMode!=245)
		{
			//change range to 245
			handle->status=TRANSMITTING;
			handle->measureMode=245;
			return i3g4250d_writeRegisterDMA(handle,CTRL_REG4,(handle->ctrl_reg4 |= 0x00));
		}
	}
	//check if all values are included in the medium sensor range
	else if(handle->x<500	&&	handle->y<500	&&	handle->z<500)
	{
		if(handle->measureMode!=500)
		{
			//change range to 500
			handle->status=TRANSMITTING;
			handle->measureMode=500;
			return i3g4250d_writeRegisterDMA(handle,CTRL_REG4,(handle->ctrl_reg4 |= 0x10));
		}
	}
	//take the biggest sensor range
	else
	{
		if(handle->measureMode!=2000)
		{
			//change range to 2000
			handle->status=TRANSMITTING;
			handle->measureMode=2000;
			return i3g4250d_writeRegisterDMA(handle,CTRL_REG4,(handle->ctrl_reg4 |= 0x20));
		}
	}
	return HAL_OK;
}

/* function:		i3g4250d_readTemperature
 * description:		read the temperature register of i3g4250d
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
HAL_StatusTypeDef i3g4250d_readTemperature(i3g4250d *handle)
{
	//check if SPI or DMA is busy
	if(handle->hdma_tx->State!=0x01||handle->hdma_tx->State!=0x01||handle->hspi->State!=0x01||handle->status!=FINISH)
	{
		handle->next=READTEMPERATURE;
		return HAL_BUSY;
	}
	handle->status=READTEMPERATURE;

	//setup SPI registers
	handle->txBuf[0]=OUT_TEMP|0x80;
	handle->txBuf[1]=0x00;

	//start SPI communication
	HAL_GPIO_WritePin(GPIOE,handle->cs_Pin,GPIO_PIN_RESET);
	return HAL_SPI_TransmitReceive_DMA(handle->hspi,&handle->txBuf[0],&handle->rxBuf[0],6);

}

/* function:		i3g4250d_calcTemperature
 * description:		calculate the temperature of i3g4250d with the raw values in the rxBuf array in the handle.
 * 			i3g4250d_readTemperatur have to be called before this function and the DMA have to been finished the SPI communication
 * 			to get correct values.
 ***************************************************************
 * *handle:		pointer to sensor handle
 ***************************************************************
 * returns:		state of transfer
 */
inline void i3g4250d_calcTemperature(i3g4250d *handle)
{
	handle->temperature=40-((int8_t)handle->rxBuf[1]);
	handle->status=FINISH;
}


