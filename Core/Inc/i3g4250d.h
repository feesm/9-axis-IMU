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

//includes
#include "stm32f3xx_hal.h"

#ifndef INRANGE
#define INRANGE(value, minValue, maxValue) (value <= maxValue \
		&& value >= minValue)
#endif
#define I3G4250D_INRANGE(phandle, minValue, maxValue) (INRANGE(phandle->x, minValue, maxValue) \
		&& INRANGE(phandle->y, minValue, maxValue) \
		&& INRANGE(phandle->z, minValue, maxValue)) \

//Read-only Registers

#define WHO_AM_I		0x0F
#define OUT_TEMP		0x26
#define STATUS_REG		0x27
#define OUT_X_L			0x28
#define OUT_X_H			0x29
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D
#define FIFO_SRC_REG		0x2F
#define INT1_SRC		0x31

//Read-Write Registers

#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define REFERENCE		0x25
#define FIFO_CTRL_REG		0x2E
#define INT1_CFG		0x30
#define INT1_THS_XH		0x32
#define INT1_THS_XL		0x33
#define INT1_THS_YH		0x34
#define INT1_THS_YL		0x35
#define INT1_THS_ZH		0x36
#define INT1_THS_ZL		0x37
#define INT1_DURATION		0x38

//SPI state
typedef enum _SensorState
{
	FINISH=0x00,
	READSENSORDATA=0x01,
	READTEMPERATURE=0x02,
	ADJUSTRANGE=0x03,
	TRANSMITTING=0x04
}SensorState;

//i3g4250d sensor handle
typedef struct _i3g4250d
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
	SensorState		status;
	SensorState		next;
}i3g4250d;


/*public function prototypes--------------------------------------------------------------------------------*/

HAL_StatusTypeDef i3g4250d_config(i3g4250d *handle, SPI_HandleTypeDef *SPI_hspi, uint16_t SPI_cs_Pin, GPIO_TypeDef * SPI_cs_Reg, DMA_HandleTypeDef *SPI_hdma_rx, DMA_HandleTypeDef *SPI_hdma_tx);
void i3g4250d_checkBlockedTask(i3g4250d *handle);
HAL_StatusTypeDef i3g4250d_readSensorData(i3g4250d *handle);
void i3g4250d_calcSensorData(i3g4250d *handle);
HAL_StatusTypeDef i3g4250d_adjustRange(i3g4250d *handle);
HAL_StatusTypeDef i3g4250d_readTemperature(i3g4250d *handle);
void i3g4250d_calcTemperature(i3g4250d *handle);


#ifdef __cplusplus
}
#endif

#endif /* INC_I3G4250D_H_ */
