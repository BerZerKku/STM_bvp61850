/*
 * wrapper.c
 *
 *  Created on: 10 авг. 2020 г.
 *      Author: Shcheblykin
 */

#include "wrapper.hpp"

#include <stdio.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void i2cReset(I2C_HandleTypeDef *hi2c);
void i2cActionStart(I2C_HandleTypeDef *hi2c, uint32_t size);
void i2cWatchDogReset();


#define I2C_TIME_RESET_MS 100
#define I2C_MAX_ERROR_COUNTER 100000

uint16_t address = 0;
uint8_t direction = 0;
uint8_t state = 0;
uint8_t buf[35] = { 0 };
bool printDebug = false;

uint32_t i2cErrorCounter = 0;
uint32_t i2cWdTimer = I2C_TIME_RESET_MS;
bool i2cAction = false;
uint32_t i2cActionTime = 0;

uint32_t debug = 0;

enum debugMsg_t {
  DEBUG_MSG_i2cWdTimerReset           = 0x0001,
  DEBUG_MSG_i2cActionTimeReset        = 0x0002,
  DEBUG_MSG_i2cErrorCounterReset      = 0x0004,
  DEBUG_MSG_HAL_I2C_AbortCpltCallback = 0x0008,
  DEBUG_MSG_i2cAddrCallback_0x3D      = 0x0010,
  DEBUG_MSG_i2cAddrCallback_0x3E      = 0x0020
};

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint16_t cnt = 0;

  if (htim == &htim6) {
    HAL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);

    if (++cnt >= 1000) {
      HAL_GPIO_TogglePin(LED1_VD7_GPIO_Port, LED1_VD7_Pin);
//      printDebug = true;
      cnt = 0;
    }

    if (i2cWdTimer > 0) {
      i2cWdTimer--;
    }

    if (i2cWdTimer == 0) {
      debug |= DEBUG_MSG_i2cWdTimerReset;
      i2cReset(&hi2c2);
    }

    if ((i2cActionTime > 0) && i2cAction) {
      i2cActionTime--;
      if (i2cActionTime == 0) {
        debug |= DEBUG_MSG_i2cActionTimeReset;
        i2cReset(&hi2c2);
      }
    }
  }
}

/**
 * @brief  Slave Address Match callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  TransferDirection Master request Transfer Direction (Write/Read),
 *                value of @ref I2C_XferDirection_definition
 * @param  AddrMatchCode Address Match Code
 * @retval None
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
    uint16_t AddrMatchCode) {
  HAL_StatusTypeDef status = HAL_ERROR;

  HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, GPIO_PIN_RESET);

  if (hi2c == &hi2c2) {
    direction = TransferDirection;
    address = AddrMatchCode >> 1;
    //    printf(" >>> direction = %d, address = %X\n", direction, address);

    if (address == 0x3D) {
      if (direction == I2C_DIRECTION_TRANSMIT) {
        status = HAL_I2C_Slave_Seq_Receive_IT(hi2c, buf, 2, I2C_LAST_FRAME);
        debug |= DEBUG_MSG_i2cAddrCallback_0x3D;
        i2cActionStart(hi2c, 2);
      } else {
        status = HAL_I2C_Slave_Seq_Transmit_IT(hi2c, buf, 35, I2C_LAST_FRAME);
        debug |= DEBUG_MSG_i2cAddrCallback_0x3D;
        i2cActionStart(hi2c, 35);
      }
    } else if (address == 0x3E) {
      if (direction == I2C_DIRECTION_TRANSMIT) {
        status = HAL_I2C_Slave_Seq_Receive_IT(hi2c, buf, 35, I2C_LAST_FRAME);
        debug |= DEBUG_MSG_i2cAddrCallback_0x3E;
        i2cActionStart(hi2c, 35);
      }
    }

    if (status != HAL_OK) {
      i2cErrorCounter++;
    }
  }

  HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, GPIO_PIN_SET);
}


/**
 * @brief  Slave Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c2) {
    if (address == 0x3D) {
      // TODO Обработчик окончания записи адреса для чтения мастером.
      // Должен занимать минимальное время, иначе не успевает начать!
      i2cWatchDogReset();
    } else if (address == 0x3E) {
      // TODO Обработчик окончания записи данных мастером.
      i2cWatchDogReset();
    }
  }
}


/**
 * @brief  Slave Tx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c2) {
    if (address == 0x3D) {
      // TODO Обработчик окончания чтения данных мастером.
      i2cWatchDogReset();
    }
  }
}


/**
 * @brief  I2C error callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c2) {
    uint32_t error = HAL_I2C_GetError(hi2c);
//    printf(" >>> Error = %lX, address = %X\n", error, address);

    i2cErrorCounter++;
    if (i2cErrorCounter >= I2C_MAX_ERROR_COUNTER) {
      debug |= DEBUG_MSG_i2cErrorCounterReset;
      i2cReset(hi2c);
    }

    if (error & HAL_I2C_ERROR_AF) {
      __HAL_I2C_CLEAR_FLAG(hi2c, HAL_I2C_ERROR_AF);
    }
  }
}


/**
 * @brief  I2C abort callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c2) {
    debug |= DEBUG_MSG_HAL_I2C_AbortCpltCallback;
  }
}


// Вызов после инициализации всей периферии в main().
void wrapperMainInit() {

  i2cWatchDogReset();
  HAL_Delay(10);
  printf("Hello STM32\n");

  HAL_GPIO_WritePin(LED2_VD8_GPIO_Port, LED2_VD8_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim6);
}


// Вызов из бесконечного цикла main().
void wrapperMainLoop() {
  HAL_IWDG_Refresh(&hiwdg);

  GPIO_PinState pinstate = GPIO_PIN_RESET;

  pinstate = HAL_GPIO_ReadPin(Sout7_GPIO_Port, Sout7_Pin);
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout6_GPIO_Port, Sout6_Pin);
  HAL_GPIO_WritePin(WARNING__GPIO_Port, WARNING__Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout5_GPIO_Port, Sout5_Pin);
  HAL_GPIO_WritePin(HF_FAULT__GPIO_Port, HF_FAULT__Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout4_GPIO_Port, Sout4_Pin);
  HAL_GPIO_WritePin(TEST_GOOSE__GPIO_Port, TEST_GOOSE__Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout2_GPIO_Port, Sout2_Pin);
  HAL_GPIO_WritePin(COM_TR_GPIO_Port, COM_TR_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout1_GPIO_Port, Sout1_Pin);
  HAL_GPIO_WritePin(COM_RC_GPIO_Port, COM_RC_Pin, pinstate);

  HAL_I2C_EnableListen_IT(&hi2c2);

  if (printDebug) {
    printf("i2c state = %X, error = %lX, mode = %X\n",
        HAL_I2C_GetState(&hi2c2), HAL_I2C_GetError(&hi2c2),
        HAL_I2C_GetMode(&hi2c2));
    printf("i2cErrorCounter=%ld\n", i2cErrorCounter);

    if (debug) {
      if (debug & DEBUG_MSG_i2cWdTimerReset) {
        printf("Reset by i2cWdTimer!\n");
        debug &= ~DEBUG_MSG_i2cWdTimerReset;
      }

      if (debug & DEBUG_MSG_i2cActionTimeReset) {
        printf("Reset by i2cActionTime!\n");
        debug &= ~DEBUG_MSG_i2cActionTimeReset;
      }

      if (debug & DEBUG_MSG_i2cErrorCounterReset) {
        printf("Reset by i2cErrorCounter!\n");
        debug &= ~DEBUG_MSG_i2cErrorCounterReset;
      }

      if (debug & DEBUG_MSG_i2cAddrCallback_0x3D) {
        printf("DEBUG_MSG_i2cAddrCallback_0x3D!\n");
        debug &= ~DEBUG_MSG_i2cAddrCallback_0x3D;
      }

      if (debug & DEBUG_MSG_i2cAddrCallback_0x3E) {
        printf("DEBUG_MSG_i2cAddrCallback_0x3E!\n");
        debug &= ~DEBUG_MSG_i2cAddrCallback_0x3E;
      }

      if (debug & DEBUG_MSG_HAL_I2C_AbortCpltCallback) {
        printf("DEBUG_MSG_HAL_I2C_AbortCpltCallback!\n");
        debug &= ~DEBUG_MSG_HAL_I2C_AbortCpltCallback;
      }
    }

    printDebug = false;
  }
}

// Сброс интерфейса I2Cю
void i2cReset(I2C_HandleTypeDef *hi2c) {
    i2cWatchDogReset();
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

void i2cWatchDogReset() {
  i2cErrorCounter = 0;
  i2cWdTimer = I2C_TIME_RESET_MS;

  i2cAction = false;
  i2cActionTime = 100;
}

void i2cActionStart(I2C_HandleTypeDef *hi2c, uint32_t size) {
  i2cActionTime = 2 + (size * 9 * 1000)/(hi2c->Init.ClockSpeed);
  i2cAction = true;
}
