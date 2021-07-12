#include <Arduino.h>

//#include "hal_conf_extra.h"
#include "stm32f7xx_hal_conf.h"//este include esta incluido en stm32f7xx_hal_.h, se debe comentar esta linea.
#include "stm32f7xx_hal.h"
//#include "stm32f7xx_it.h"
SPI_HandleTypeDef hspi6;
uint32_t ledpin = PB0;
uint32_t CS = PA4;

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hspi->Instance == SPI6)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI6_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI6 GPIO Configuration
    PB3     ------> SPI6_SCK
    PB4     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI6)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI6_CLK_DISABLE();

    /**SPI6 GPIO Configuration
    PB3     ------> SPI6_SCK
    PB4     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
  }
}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_SLAVE;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 7;
  hspi6.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
}


//void Error_Handler()
//{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
 // __disable_irq();
//  while (1)
//  {
 // }
  /* USER CODE END Error_Handler_Debug */
//}

void DSA_Transmit(void)
{
  uint8_t data[2]={2,1};

  if (/*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET*/!digitalRead(CS))
  {
  HAL_SPI_Transmit(&hspi6,data,2,10);
  Serial.println("se ha seleccionado el esclavo");
  }else{Serial.println("no se ha seleccionado el esclavo");}
}


//RCC->AHB1ENR =0;
//GPIOA->MODER |= (1 << 1) | (1 << 0);
//GPIOA->OTYPER =0;

void setup() 
{
  Serial.begin(115200);
  Serial.flush();
  //HAL_Init();
  MX_SPI6_Init();
  pinMode(ledpin,OUTPUT);
  pinMode(CS,   INPUT);
}

void loop() 
{
  DSA_Transmit();
  digitalWrite(ledpin,HIGH);
  delay(500);
  digitalWrite(ledpin,LOW);
  delay(500);
 // Serial.println("no se ha seleccionado el esclavo");
}