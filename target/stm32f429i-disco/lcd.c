/*
 * Copyright (c) 2015 Travis Geiselbrecht
 * Copyright (c) 2022 Luka Panio
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
/*
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include <assert.h>
#include <lk/err.h>
#include <lk/debug.h>
#include <lk/trace.h>
#include <target.h>
#include <lk/compiler.h>
#include <string.h>
#include <dev/gpio.h>
#include <dev/display.h>
#include <kernel/timer.h>
#include <arch/ops.h>
#include <platform/stm32.h>
#include <platform/gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_ltdc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_spi.h>
#include <target/lcd.h>

/*
 * lcd initialization sequence, taken from
 * STM32F429I-Discovery_FW_V1.0.3/Utilities/STM32F429I-Discovery/stm32f429i_discovery_lcd.[ch]
 */

/**
  * @brief  Controls LCD Chip Select (CS) pin
  * @param  NewState CS pin state
  * @retval None
  */
void STM_LCD_ChipSelect(FunctionalState NewState)
{
  if (NewState == DISABLE)
  {
    GPIO_ResetBits(LCD_NCS_GPIO_PORT, LCD_NCS_PIN); /* CS pin low: LCD disabled */
  }
  else
  {
    GPIO_SetBits(LCD_NCS_GPIO_PORT, LCD_NCS_PIN); /* CS pin high: LCD enabled */
  }
}

/**
  * @brief  Configures the LCD_SPI interface.
  * @param  None
  * @retval None
  */
void STM_LCD_SPIConfig(void)
{
  SPI_InitTypeDef    SPI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable LCD_SPI_SCK_GPIO_CLK, LCD_SPI_MISO_GPIO_CLK and LCD_SPI_MOSI_GPIO_CLK clock */
  RCC_AHB1PeriphClockCmd(LCD_SPI_SCK_GPIO_CLK | LCD_SPI_MISO_GPIO_CLK | LCD_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable LCD_SPI and SYSCFG clock  */
  RCC_APB2PeriphClockCmd(LCD_SPI_CLK, ENABLE);
  
  /* Configure LCD_SPI SCK pin */
  GPIO_InitStructure.GPIO_Pin = LCD_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(LCD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_SPI MISO pin */
  GPIO_InitStructure.GPIO_Pin = LCD_SPI_MISO_PIN;
  GPIO_Init(LCD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_SPI MOSI pin */
  GPIO_InitStructure.GPIO_Pin = LCD_SPI_MOSI_PIN;
  GPIO_Init(LCD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* Connect SPI SCK */
  GPIO_PinAFConfig(LCD_SPI_SCK_GPIO_PORT, LCD_SPI_SCK_SOURCE, LCD_SPI_SCK_AF);

  /* Connect SPI MISO */
  GPIO_PinAFConfig(LCD_SPI_MISO_GPIO_PORT, LCD_SPI_MISO_SOURCE, LCD_SPI_MISO_AF);

  /* Connect SPI MOSI */
  GPIO_PinAFConfig(LCD_SPI_MOSI_GPIO_PORT, LCD_SPI_MOSI_SOURCE, LCD_SPI_MOSI_AF);
  
  SPI_I2S_DeInit(LCD_SPI);

  /* SPI configuration -------------------------------------------------------*/
  /* If the SPI peripheral is already enabled, don't reconfigure it */
  if ((LCD_SPI->CR1 & SPI_CR1_SPE) == 0)
  {    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz) 
       to verify these constraints:
          - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
          - l3gd20 SPI interface max baudrate is 10MHz for write/read
          - PCLK2 frequency is set to 90 MHz 
       */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(LCD_SPI, &SPI_InitStructure);

    /* Enable L3GD20_SPI  */
    SPI_Cmd(LCD_SPI, ENABLE);
  }
}
 
static void STM_LCD_AF_GPIOConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOF, GPIOG AHB Clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | \
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | \
                         RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

/* GPIOs Configuration */
/*
 +------------------------+-----------------------+----------------------------+
 +                       LCD pins assignment                                   +
 +------------------------+-----------------------+----------------------------+
 |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
 |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
 |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
 |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
 |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
 |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
 -------------------------------------------------------------------------------
          |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
          |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
           -----------------------------------------------------

*/

 /* GPIOA configuration */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_LTDC);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | \
                             GPIO_Pin_11 | GPIO_Pin_12;
                             
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
 /* GPIOB configuration */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, 0x09);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, 0x09);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LTDC);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | \
                             GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  
  GPIO_Init(GPIOB, &GPIO_InitStruct);

 /* GPIOC configuration */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10;
                             
  GPIO_Init(GPIOC, &GPIO_InitStruct);

 /* GPIOD configuration */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6;
                             
  GPIO_Init(GPIOD, &GPIO_InitStruct);
  
 /* GPIOF configuration */
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
                             
  GPIO_Init(GPIOF, &GPIO_InitStruct);     

 /* GPIOG configuration */  
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource6, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource7, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, 0x09);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_LTDC);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, 0x09);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | \
                             GPIO_Pin_11 | GPIO_Pin_12;
  
  GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
  * @brief  Sets or reset LCD control lines.
  * @param  GPIOx: where x can be B or D to select the GPIO peripheral.
  * @param  CtrlPins: the Control line.
  *   This parameter can be:
  *     @arg LCD_NCS_PIN: Chip Select pin
  *     @arg LCD_NWR_PIN: Read/Write Selection pin
  *     @arg LCD_RS_PIN: Register/RAM Selection pin
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void STM_LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, uint16_t CtrlPins, BitAction BitVal)
{
  /* Set or Reset the control line */
  GPIO_WriteBit(GPIOx, (uint16_t)CtrlPins, (BitAction)BitVal);
}

/**
  * @brief  Configures LCD control lines in Output Push-Pull mode.
  * @note   The LCD_NCS line can be configured in Open Drain mode  
  *         when VDDIO is lower than required LCD supply.
  * @param  None
  * @retval None
  */
void STM_LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clock*/
  RCC_AHB1PeriphClockCmd(LCD_NCS_GPIO_CLK | LCD_WRX_GPIO_CLK, ENABLE);

  /* Configure NCS in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_NCS_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure WRX in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_WRX_PIN;
  GPIO_Init(LCD_WRX_GPIO_PORT, &GPIO_InitStructure);

  /* Set chip select pin high */
  STM_LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}

 /**
  * @brief  Writes command to select the LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void STM_LCD_WriteCommand(uint8_t LCD_Reg)
{
    /* Reset WRX to send command */
  STM_LCD_CtrlLinesWrite(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, Bit_RESET);
  
  /* Reset LCD control line(/CS) and Send command */
  STM_LCD_ChipSelect(DISABLE);
  SPI_I2S_SendData(LCD_SPI, LCD_Reg);
  
  /* Wait until a data is sent(not busy), before config /CS HIGH */
  
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET) ;
  
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);
  
  STM_LCD_ChipSelect(ENABLE);
}

/**
  * @brief  Writes data to select the LCD register.
  *         This function must be used after STM_LCD_WriteCommand() function
  * @param  value: data to write to the selected register.
  * @retval None
  */
void STM_LCD_WriteData(uint8_t value)
{
    /* Set WRX to send data */
  STM_LCD_CtrlLinesWrite(LCD_WRX_GPIO_PORT, LCD_WRX_PIN, Bit_SET);
  
  /* Reset LCD control line(/CS) and Send data */  
  STM_LCD_ChipSelect(DISABLE);
  SPI_I2S_SendData(LCD_SPI, value);
  
  /* Wait until a data is sent(not busy), before config /CS HIGH */
  
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET) ;
  
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);
  
  STM_LCD_ChipSelect(ENABLE);
}

/**
  * @brief  Configure the LCD controller (Power On sequence as described in ILI9341 Datasheet)
  * @param  None
  * @retval None
  */
void STM_LCD_PowerOn(void)
{
  STM_LCD_WriteCommand(0xCA);
  STM_LCD_WriteData(0xC3);
  STM_LCD_WriteData(0x08);
  STM_LCD_WriteData(0x50);
  STM_LCD_WriteCommand(LCD_POWERB);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0xC1);
  STM_LCD_WriteData(0x30);
  STM_LCD_WriteCommand(LCD_POWER_SEQ);
  STM_LCD_WriteData(0x64);
  STM_LCD_WriteData(0x03);
  STM_LCD_WriteData(0x12);
  STM_LCD_WriteData(0x81);
  STM_LCD_WriteCommand(LCD_DTCA);
  STM_LCD_WriteData(0x85);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x78);
  STM_LCD_WriteCommand(LCD_POWERA);
  STM_LCD_WriteData(0x39);
  STM_LCD_WriteData(0x2C);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x34);
  STM_LCD_WriteData(0x02);
  STM_LCD_WriteCommand(LCD_PRC);
  STM_LCD_WriteData(0x20);
  STM_LCD_WriteCommand(LCD_DTCB);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteCommand(LCD_FRC);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x1B);
  STM_LCD_WriteCommand(LCD_DFC);
  STM_LCD_WriteData(0x0A);
  STM_LCD_WriteData(0xA2);
  STM_LCD_WriteCommand(LCD_POWER1);
  STM_LCD_WriteData(0x10);
  STM_LCD_WriteCommand(LCD_POWER2);
  STM_LCD_WriteData(0x10);
  STM_LCD_WriteCommand(LCD_VCOM1);
  STM_LCD_WriteData(0x45);
  STM_LCD_WriteData(0x15);
  STM_LCD_WriteCommand(LCD_VCOM2);
  STM_LCD_WriteData(0x90);
  STM_LCD_WriteCommand(LCD_MAC);
  STM_LCD_WriteData(0xC8);
  STM_LCD_WriteCommand(LCD_3GAMMA_EN);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteCommand(LCD_RGB_INTERFACE);
  STM_LCD_WriteData(0xC2);
  STM_LCD_WriteCommand(LCD_DFC);
  STM_LCD_WriteData(0x0A);
  STM_LCD_WriteData(0xA7);
  STM_LCD_WriteData(0x27);
  STM_LCD_WriteData(0x04);

  /* colomn address set */
  STM_LCD_WriteCommand(LCD_COLUMN_ADDR);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0xEF);
  /* Page Address Set */
  STM_LCD_WriteCommand(LCD_PAGE_ADDR);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x01);
  STM_LCD_WriteData(0x3F);
  STM_LCD_WriteCommand(LCD_INTERFACE);
  STM_LCD_WriteData(0x01);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x06);
  
  STM_LCD_WriteCommand(LCD_GRAM);
  thread_sleep(200);
  
  STM_LCD_WriteCommand(LCD_GAMMA);
  STM_LCD_WriteData(0x01);
  
  STM_LCD_WriteCommand(LCD_PGAMMA);
  STM_LCD_WriteData(0x0F);
  STM_LCD_WriteData(0x29);
  STM_LCD_WriteData(0x24);
  STM_LCD_WriteData(0x0C);
  STM_LCD_WriteData(0x0E);
  STM_LCD_WriteData(0x09);
  STM_LCD_WriteData(0x4E);
  STM_LCD_WriteData(0x78);
  STM_LCD_WriteData(0x3C);
  STM_LCD_WriteData(0x09);
  STM_LCD_WriteData(0x13);
  STM_LCD_WriteData(0x05);
  STM_LCD_WriteData(0x17);
  STM_LCD_WriteData(0x11);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteCommand(LCD_NGAMMA);
  STM_LCD_WriteData(0x00);
  STM_LCD_WriteData(0x16);
  STM_LCD_WriteData(0x1B);
  STM_LCD_WriteData(0x04);
  STM_LCD_WriteData(0x11);
  STM_LCD_WriteData(0x07);
  STM_LCD_WriteData(0x31);
  STM_LCD_WriteData(0x33);
  STM_LCD_WriteData(0x42);
  STM_LCD_WriteData(0x05);
  STM_LCD_WriteData(0x0C);
  STM_LCD_WriteData(0x0A);
  STM_LCD_WriteData(0x28);
  STM_LCD_WriteData(0x2F);
  STM_LCD_WriteData(0x0F);
  
  STM_LCD_WriteCommand(LCD_SLEEP_OUT);
  thread_sleep(200);
  STM_LCD_WriteCommand(LCD_DISPLAY_ON);
  /* GRAM start writing */
  STM_LCD_WriteCommand(LCD_GRAM);
 }
 
/**
  * @brief LCD Configuration.
  * @note  This function Configure tha LTDC peripheral :
  *        1) Configure the Pixel Clock for the LCD
  *        2) Configure the LTDC Timing and Polarity
  *        3) Configure the LTDC Layer 1 :
  *           - The frame buffer is located at FLASH memory
  *           - The Layer size configuration : 480x272                      
  * @retval
  *  None
  */
static void STM_LCD_Config(void)
{
  LTDC_InitTypeDef       LTDC_InitStruct;

  /* Configure the LCD Control pins ------------------------------------------*/
  STM_LCD_CtrlLinesConfig();
  STM_LCD_ChipSelect(DISABLE);
  STM_LCD_ChipSelect(ENABLE);
  
  /* Configure the LCD_SPI interface -----------------------------------------*/
  STM_LCD_SPIConfig(); 
  
  /* Power on the LCD --------------------------------------------------------*/
  STM_LCD_PowerOn();
  
  /* Enable the LTDC Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, ENABLE);
  
  /* Enable the DMA2D Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE); 

/* Configure the LCD Control pins --------------------------------------------*/
  STM_LCD_AF_GPIOConfig();
      
/* LTDC Configuration *********************************************************/  
  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  LTDC_InitStruct.LTDC_HSPolarity = LTDC_HSPolarity_AL;     
  /* Initialize the vertical synchronization polarity as active low */  
  LTDC_InitStruct.LTDC_VSPolarity = LTDC_VSPolarity_AL;     
  /* Initialize the data enable polarity as active low */
  LTDC_InitStruct.LTDC_DEPolarity = LTDC_DEPolarity_AL;     
  /* Initialize the pixel clock polarity as input pixel clock */ 
  LTDC_InitStruct.LTDC_PCPolarity = LTDC_PCPolarity_IPC;
  
  /* Configure R,G,B component values for LCD background color */                   
  LTDC_InitStruct.LTDC_BackgroundRedValue = 0;            
  LTDC_InitStruct.LTDC_BackgroundGreenValue = 0;          
  LTDC_InitStruct.LTDC_BackgroundBlueValue = 0;  
  
  /* Configure PLLSAI prescalers for LCD */
  /* Enable Pixel Clock */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAI_N = 192 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAI_R = 192/4 = 48 Mhz */
  /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDivR = 48/8 = 6 Mhz */
  RCC_PLLSAIConfig(192, 7, 4);
  RCC_LTDCCLKDivConfig(RCC_PLLSAIDivR_Div8);
  
  /* Enable PLLSAI Clock */
  RCC_PLLSAICmd(ENABLE);
  /* Wait for PLLSAI activation */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLSAIRDY) == RESET)
  {
  }
  
  /* Timing configuration */  
  /* Configure horizontal synchronization width */     
  LTDC_InitStruct.LTDC_HorizontalSync = 9;
  /* Configure vertical synchronization height */
  LTDC_InitStruct.LTDC_VerticalSync = 1;
  /* Configure accumulated horizontal back porch */
  LTDC_InitStruct.LTDC_AccumulatedHBP = 29; 
  /* Configure accumulated vertical back porch */
  LTDC_InitStruct.LTDC_AccumulatedVBP = 3;  
  /* Configure accumulated active width */  
  LTDC_InitStruct.LTDC_AccumulatedActiveW = 269;
  /* Configure accumulated active height */
  LTDC_InitStruct.LTDC_AccumulatedActiveH = 323;
  /* Configure total width */
  LTDC_InitStruct.LTDC_TotalWidth = 279; 
  /* Configure total height */
  LTDC_InitStruct.LTDC_TotalHeigh = 327;
  
  LTDC_Init(&LTDC_InitStruct);
  
}

/**
  * @brief  Initializes the LCD Layers.
  * @param  None
  * @retval None
  */
void LCD_LayerInit(void)
{
  LTDC_Layer_InitTypeDef LTDC_Layer_InitStruct; 
  
  /* Windowing configuration */
  /* In this case all the active display area is used to display a picture then :
  Horizontal start = horizontal synchronization + Horizontal back porch = 30 
  Horizontal stop = Horizontal start + window width -1 = 30 + 240 -1
  Vertical start   = vertical synchronization + vertical back porch     = 4
  Vertical stop   = Vertical start + window height -1  = 4 + 320 -1      */      
  LTDC_Layer_InitStruct.LTDC_HorizontalStart = 30;
  LTDC_Layer_InitStruct.LTDC_HorizontalStop = (LCD_SIZE_PIXEL_WIDTH + 30 - 1); 
  LTDC_Layer_InitStruct.LTDC_VerticalStart = 4;
  LTDC_Layer_InitStruct.LTDC_VerticalStop = (LCD_SIZE_PIXEL_HEIGHT + 4 - 1);
  
  /* Pixel Format configuration*/
  LTDC_Layer_InitStruct.LTDC_PixelFormat = LTDC_Pixelformat_ARGB8888;
  /* Alpha constant (255 totally opaque) */
  LTDC_Layer_InitStruct.LTDC_ConstantAlpha = 255; 
  /* Default Color configuration (configure A,R,G,B component values) */          
  LTDC_Layer_InitStruct.LTDC_DefaultColorBlue = 0;        
  LTDC_Layer_InitStruct.LTDC_DefaultColorGreen = 0;       
  LTDC_Layer_InitStruct.LTDC_DefaultColorRed = 0;         
  LTDC_Layer_InitStruct.LTDC_DefaultColorAlpha = 0;
  /* Configure blending factors */       
  LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_CA;    
  
  /* the length of one line of pixels in bytes + 3 then :
  Line Lenth = Active high width x number of bytes per pixel + 3 
  Active high width         = LCD_SIZE_PIXEL_WIDTH 
  number of bytes per pixel = 4    (pixel_format : ARGB8888) 
  */
  LTDC_Layer_InitStruct.LTDC_CFBLineLength = ((LCD_SIZE_PIXEL_WIDTH * 4) + 3);
  /* the pitch is the increment from the start of one line of pixels to the 
  start of the next line in bytes, then :
  Pitch = Active high width x number of bytes per pixel */ 
  LTDC_Layer_InitStruct.LTDC_CFBPitch = (LCD_SIZE_PIXEL_WIDTH * 4);
  
  /* Configure the number of lines */  
  LTDC_Layer_InitStruct.LTDC_CFBLineNumber = LCD_SIZE_PIXEL_HEIGHT;
  
  /* Start Address configuration : the LCD Frame buffer is defined on SDRAM */    
  LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER;
  
  /* Initialize LTDC layer 1 */
  LTDC_LayerInit(LTDC_Layer1, &LTDC_Layer_InitStruct);
  
  /* Configure blending factors */       
  LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;    
  
  /* LTDC configuration reload */  
  LTDC_ReloadConfig(LTDC_IMReload);
  
  /* Enable foreground & background Layers */
  LTDC_LayerCmd(LTDC_Layer1, ENABLE); 
  
  /* LTDC configuration reload */  
  LTDC_ReloadConfig(LTDC_IMReload);
  
  /* dithering activation */
  LTDC_DitherCmd(ENABLE);
}

uint8_t STM_LCD_Init(void) {
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f429_439xx.s) before to branch to application main.
     */     

  dprintf(SPEW, "initializing LCD\n");

  /* Configure LCD : Only one layer is used */
  STM_LCD_Config();

  /* Enable Layer 1 */
  LTDC_LayerCmd(LTDC_Layer1, ENABLE);
  
  /* Reload configuration of Layer 1 */
  LTDC_ReloadConfig(LTDC_IMReload);
  
  /* Enable The LCD */
  LTDC_Cmd(ENABLE);
  
  /* Setup layers */
  LCD_LayerInit();

    return 0;
}

/* LK display api here */
status_t display_get_framebuffer(struct display_framebuffer *fb) {
   /* fb->image.pixels = (void *)hLtdcEval.LayerCfg[ActiveLayer].FBStartAdress;

    if (hLtdcEval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888) {
        fb->format = DISPLAY_FORMAT_ARGB_8888;
        fb->image.format = IMAGE_FORMAT_ARGB_8888;
        fb->image.rowbytes = BSP_LCD_GetXSize() * 4;
    } else if (hLtdcEval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) {
        fb->format = DISPLAY_FORMAT_RGB_565;
        fb->image.format = IMAGE_FORMAT_RGB_565;
        fb->image.rowbytes = BSP_LCD_GetXSize() * 2;
    } else {
        panic("unhandled pixel format\n");
        return ERR_NOT_FOUND;
    }

    fb->image.width = BSP_LCD_GetXSize();
    fb->image.height = BSP_LCD_GetYSize();
    fb->image.stride = BSP_LCD_GetXSize();
    fb->flush = NULL;*/

    return NO_ERROR;
}

status_t display_get_info(struct display_info *info) {
    info->format = DISPLAY_FORMAT_ARGB_8888;
    info->width = LCD_SIZE_PIXEL_WIDTH;
    info->height = LCD_SIZE_PIXEL_HEIGHT;

    return NO_ERROR;
}

status_t display_present(struct display_image *image, uint starty, uint endy) {
    TRACEF("display_present - not implemented");
    DEBUG_ASSERT(false);
    return NO_ERROR;
}
