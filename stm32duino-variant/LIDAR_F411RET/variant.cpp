/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
const PinName digitalPin[] = {
  PA_0,   //D0 USART 2 RX
  PA_1,   //D1 USART 2 TX
  PA_2,   //D2 USART 1 RX
  PA_3,   //D3 USART 1 TX
  PA_4,   //D4 USB D-
  PA_5,   //D5 USB D+
  PA_6,   //D6 SCL IMU
  PA_7,   //D7 SDA IMU
  PA_8,   //D8 INT IMU
  PA_9,   //D9 EXT_CS (unused)
  PA_10,  //D10
  PA_11,  //D11
  PA_12,  //D12
  PA_13,  //D13
  PA_14,  //D14
  PA_15,  //D15

  PB_0,   //D16
  PB_1,   //D17
  PB_2,   //D18
  PB_3,   //D19
  PB_4,   //D20
  PB_5,   //D21
  PB_6,   //D22
  PB_7,   //D23
  PB_8,   //D24
  PB_9,   //D25
  PB_10,  //D26
  PB_11,  //D27
  PB_12,  //D28
  PB_13,  //D29
  PB_14,  //D30
  PB_15,  //D31

  PC_0,   //D32
  PC_1,   //D33
  PC_2,   //D34
  PC_3,   //D35
  PC_4,   //D36
  PC_5,   //D37
  PC_6,   //D38
  PC_7,   //D39
  PC_8,   //D40
  PC_9,   //D41
  PC_10,  //D42
  PC_11,  //D43
  PC_12,  //D44
  PC_13,  //D45
  PC_14,  //D46/A0
  PC_15,  //D47/A1
  
  // Duplicated pins in order to be aligned with PinMapADC
  PC_11,  //D48/A2
  PC_11,  //D49/A3
  PC_11,  //D50/A4
  PC_11,  //D51/A5
  PC_11,  //D52/A6  = D11
  PC_11,  //D53/A7  = D12
  PC_11,  //D54/A8  = D28
  PC_11,  //D55/A9  = D29
  PC_11,  //D56/A10 = D35
  PC_11,  //D57/A11 = D41
  PC_11   //D58/A12 = D45
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 10000000
  *            HCLK(Hz)                       = 10000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 8
  *            PLL_N                          = 100
  *            PLL_P                          = 4
  *            PLL_Q                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Configure the Systick interrupt time */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /* Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}
#ifdef __cplusplus
}
#endif
