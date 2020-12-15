/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: 9 בדצמ׳ 2020
 *      Author: ITSIK
 */

#include "stm32f4xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DE();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DE();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DE();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DE();
		}
	}
}
