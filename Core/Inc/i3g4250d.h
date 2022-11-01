/*
 * i3g4250d.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Moritz
 */

#ifndef INC_I3G4250D_H_
#define INC_I3G4250D_H_

#ifdef __cplusplus
extern "C" {
#endif

/*i3g4250d includes-----------------------------------------------------------------------------------*/

#include "stm32f3xx_hal.h"

/*i3g4250d macros-------------------------------------------------------------------------------------*/

#define I3G4250D_READY(phandle)  (phandle->currentTask==i3g4250d_NONE \
		&& phandle->hspi->State == HAL_SPI_STATE_READY \
		&& phandle->hdma_tx->State == HAL_DMA_STATE_READY \
		&& phandle->hdma_rx->State == HAL_DMA_STATE_READY)
#ifndef INRANGE
#define INRANGE(value, minValue, maxValue) (value <= maxValue \
		&& value >= minValue)
#endif
#define I3G4250D_INRANGE(phandle, minValue, maxValue) (INRANGE(phandle->x, minValue, maxValue) \
		&& INRANGE(phandle->y, minValue, maxValue) \
		&& INRANGE(phandle->z, minValue, maxValue)) \

/*i3g4250d registers-----------------------------------------------------------------------------------*/

#define WHO_AM_I		0x0F
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define REFERENCE		0x25
#define OUT_TEMP		0x26
#define STATUS_REG		0x27
#define OUT_X_L			0x28
#define OUT_X_H			0x29
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT1_CFG		0x30
#define INT1_SRC		0x31
#define INT1_THS_XH		0x32
#define INT1_THS_XL		0x33
#define INT1_THS_YH		0x34
#define INT1_THS_YL		0x35
#define INT1_THS_ZH		0x36
#define INT1_THS_ZL		0x37
#define INT1_DURATION		0x38

/*i3g4250d constants-------------------------------------------------------------------------------------*/

#define I3G4250D_TASKQUEUESIZE 3

#define DEFAULT_CTRL_REG1 	0xFF
#define DEFAULT_CTRL_REG2 	0x00
#define DEFAULT_CTRL_REG3 	0x08
#define DEFAULT_CTRL_REG4 	0x00
#define DEFAULT_CTRL_REG5 	0x00

/*i3g4250d typedefs--------------------------------------------------------------------------------------*/

typedef enum _i3g4250d_sensorTask
{
	i3g4250d_NONE = 0x00,
	i3g4250d_GETANGULARRATE = 0x10,
	i3g4250d_GETTEMPERATURE = 0x11,
	i3g4250d_TRANSMITTING = 0x20,
	i3g4250d_CHANGEANGRANGE = 0x30,
	i3g4250d_CHANGEANGRANGE245DPS = 0x31,
	i3g4250d_CHANGEANGRANGE500DPS = 0x32,
	i3g4250d_CHANGEANGRANGE2000DPS = 0x33,

}i3g4250d_sensorTask;


typedef struct _i3g4250d	//i3g4250d sensor handle
{
	SPI_HandleTypeDef	*hspi;
	DMA_HandleTypeDef	*hdma_rx;
	DMA_HandleTypeDef	*hdma_tx;
	uint16_t		cs_Pin;
	GPIO_TypeDef		*cs_Reg;
	uint8_t			rxBuf[7];
	uint8_t			txBuf[7];
	float			x;
	float			y;
	float			z;
	int8_t			temperature;
	uint16_t		measureMode;
	uint8_t			ctrl_reg4;
	i3g4250d_sensorTask	currentTask;
	i3g4250d_sensorTask	nextTask[I3G4250D_TASKQUEUESIZE];
}i3g4250d;


/*public function prototypes--------------------------------------------------------------------------------*/

HAL_StatusTypeDef i3g4250d_config(i3g4250d *handle, SPI_HandleTypeDef *SPI_hspi, uint16_t SPI_cs_Pin, GPIO_TypeDef * SPI_cs_Reg, DMA_HandleTypeDef *SPI_hdma_rx, DMA_HandleTypeDef *SPI_hdma_tx);
HAL_StatusTypeDef i3g4250d_readSensorData(i3g4250d *handle);
void i3g4250d_calcSensorData(i3g4250d *handle);
HAL_StatusTypeDef i3g4250d_readTemperature(i3g4250d *handle);
void i3g4250d_calcTemperature(i3g4250d *handle);
void i3g4250d_startNextTask(i3g4250d *handle);


#ifdef __cplusplus
}
#endif

#endif /* INC_I3G4250D_H_ */
