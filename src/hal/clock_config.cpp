#include <Arduino.h>

// Override the weak SystemClock_Config from the Arduino core.
// Uses 8MHz HSE instead of HSI for improved CAN bit timing accuracy.
// HSE(8MHz) / PLLM(4) * PLLN(192) / PLLP(4) = 96MHz SYSCLK
// APB1 = 48MHz, APB2 = 96MHz, USB = 48MHz — identical to HSI config.
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 4;
  RCC_OscInitStruct.PLL.PLLN       = 192;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ       = 8;
  RCC_OscInitStruct.PLL.PLLR       = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   // 96MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     // 48MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     // 96MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}
