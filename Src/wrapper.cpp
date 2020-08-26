/*
 * wrapper.c
 *
 *  Created on: 10 авг. 2020 г.
 *      Author: Shcheblykin
 */


#include "wrapper.hpp"

#include <stdio.h>
#include "debug.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

static void i2cReset(I2C_HandleTypeDef *hi2c);
static void i2cActionStart(I2C_HandleTypeDef *hi2c, uint32_t size);
static void i2cActionStop();
static void i2cWatchDogReset();

static void powerWatchDogOff();
static void powerWatchDog();

static void rpiWatchDog();
static void rpiWatchDogReset();
static void rpiReset();

#define I2C_TIME_RESET_MS 100
#define I2C_MAX_ERROR_COUNTER 5

#define POWER_OFF_TIME_MS 100

#define RPI_REBOOT_TIME_MS 30000
#define RPI_RESET_NO_CONNECT_MS 2000

uint16_t address = 0;
uint8_t direction = 0;
uint8_t state = 0;
uint8_t buf[36] = { 0 };

volatile bool i2cAction = false;
uint32_t i2cActionTime = 0;
uint32_t i2cErrorCounter = 0;
uint32_t i2cErrorCounterAf = 0;
uint32_t i2cWdTimer = I2C_TIME_RESET_MS;
uint64_t data = 0;

bool power = true;
uint32_t powerOffTime = POWER_OFF_TIME_MS;

bool rpiConnection = false;
uint32_t rpiTimeToReset = RPI_REBOOT_TIME_MS;

bool printDebug = true;
uint32_t debug = 0;

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint16_t cnt = 0;

  if (htim == &htim6) {
    HAL_GPIO_TogglePin(TP2_GPIO_Port, TP2_Pin);
    HAL_GPIO_WritePin(LED2_VD8_GPIO_Port, LED2_VD8_Pin,
        rpiConnection ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (++cnt >= 1000) {
      HAL_GPIO_TogglePin(LED1_VD7_GPIO_Port, LED1_VD7_Pin);
      Debug::proc();
      cnt = 0;
    }

    if (i2cWdTimer > 0) {
      i2cWdTimer--;
    }

    if (i2cWdTimer == 0) {
      Debug::addMsg(Debug::MSG_i2cWdTimerReset);
      i2cReset(&hi2c2);
    }

    if ((i2cActionTime > 0) && i2cAction) {
      if (--i2cActionTime == 0) {
        Debug::addMsg(Debug::MSG_i2cActionTimeReset);
        i2cReset(&hi2c2);
      }
    }

    if (powerOffTime > 0) {
      if (--powerOffTime == 0) {
        powerWatchDogOff();
      }
    }

    powerWatchDog();
    rpiWatchDog();
    Debug::send();
  }
}


/**
 * @brief  Slave Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c2) {
    i2cActionStop();

    if (HAL_I2C_Slave_Transmit_IT(hi2c, buf, 36) != HAL_BUSY) {
      i2cActionStart(&hi2c2, 36);
    }

    rpiWatchDogReset();
    i2cWatchDogReset();
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
    i2cActionStop();

    if (HAL_I2C_Slave_Receive_IT(&hi2c2, buf, 36) != HAL_BUSY) {
      i2cActionStart(&hi2c2, 36);
    }

    rpiWatchDogReset();
    i2cWatchDogReset();
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

    Debug::addMsg(Debug::MSG_i2cErrorCallback);
    if (error & HAL_I2C_ERROR_AF) {
      Debug::addMsg(Debug::MSG_i2cErrorCounterAf);
    }

    i2cErrorCounter++;
    i2cActionStop();
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
    Debug::addMsg(Debug::MSG_HAL_I2C_AbortCpltCallback);
  }
}


// Вызов после инициализации всей периферии в main().
void wrapperMainInit() {

  i2cWatchDogReset();
  HAL_Delay(10);
  printf("Hello STM32\n");

  HAL_TIM_Base_Start_IT(&htim6);
}


// Вызов из бесконечного цикла main().
void wrapperMainLoop() {
  HAL_IWDG_Refresh(&hiwdg);

  GPIO_PinState pinstate = GPIO_PIN_RESET;

  pinstate = HAL_GPIO_ReadPin(Sout7_GPIO_Port, Sout7_Pin);
  pinstate = rpiConnection ? pinstate : GPIO_PIN_SET;

  if (!rpiConnection) {
    Debug::addMsg(Debug::MSG_rpiConnectionNo);
  }

  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout6_GPIO_Port, Sout6_Pin);
  HAL_GPIO_WritePin(WARNING_GPIO_Port, WARNING_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout5_GPIO_Port, Sout5_Pin);
  HAL_GPIO_WritePin(HF_FAULT_GPIO_Port, HF_FAULT_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout4_GPIO_Port, Sout4_Pin);
  HAL_GPIO_WritePin(TEST_GOOSE_GPIO_Port, TEST_GOOSE_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout2_GPIO_Port, Sout2_Pin);
  HAL_GPIO_WritePin(COM_TR_GPIO_Port, COM_TR_Pin, pinstate);

  pinstate = HAL_GPIO_ReadPin(Sout1_GPIO_Port, Sout1_Pin);
  HAL_GPIO_WritePin(COM_RC_GPIO_Port, COM_RC_Pin, pinstate);

//  HAL_I2C_EnableListen_IT(&hi2c2);
  if (HAL_I2C_Slave_Receive_IT(&hi2c2, buf, 36) != HAL_BUSY) {
    i2cActionStart(&hi2c2, 36);
  }
}

// Сброс интерфейса I2Cю
void i2cReset(I2C_HandleTypeDef *hi2c) {
    i2cWatchDogReset();

    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

//
void i2cWatchDogReset() {
  i2cWdTimer = I2C_TIME_RESET_MS;

  i2cAction = false;
  i2cActionTime = 100;
}

//
void i2cActionStart(I2C_HandleTypeDef *hi2c, uint32_t size) {
  // Минимальное время на действие.
  i2cActionTime = 5;
  // 9 = 8 data bits + 1 ack/nack, старт и стоп биты по одному на всю посылку.
  i2cActionTime += (size * 9 * 1000)/(hi2c->Init.ClockSpeed);
  i2cAction = true;

}

//
void i2cActionStop() {
  i2cAction = false;
  i2cActionTime = 100;
}

/// Сторожевой таймер для Raspberry.
void rpiWatchDog() {
  if (rpiTimeToReset > 0) {
    rpiTimeToReset--;
  }

  /// Сигнал сброса подается в течении одного такта таймера.
  if (rpiTimeToReset == 0) {
    rpiReset();
    rpiTimeToReset = RPI_REBOOT_TIME_MS;
  } else {
    HAL_GPIO_WritePin(RASP_RESET_GPIO_Port, RASP_RESET_Pin, GPIO_PIN_SET);
  }
}

//
void rpiWatchDogReset() {
  // FIXME сделать сброс только для корректно принятых пакетов!
  rpiConnection = true;
  rpiTimeToReset = RPI_RESET_NO_CONNECT_MS;
}

//
void rpiReset() {
#ifdef NDEBUG
  HAL_GPIO_WritePin(RASP_RESET_GPIO_Port, RASP_RESET_Pin, GPIO_PIN_RESET);
#endif
  rpiConnection = false;
  Debug::addMsg(Debug::MSG_rpiReset);
}

// TODO переделать на прерывание ?!
void powerWatchDog() {
  GPIO_PinState pinstate = GPIO_PIN_RESET;

  pinstate = HAL_GPIO_ReadPin(EXT_PWR_DOWN_GPIO_Port, EXT_PWR_DOWN_Pin);
  if (pinstate == GPIO_PIN_SET) {
    powerOffTime = POWER_OFF_TIME_MS;
  } else {
    Debug::addMsg(Debug::MSG_powerExtPwrDownIsLow);
  }
}

//
void powerWatchDogOff() {
  // При выключении необходимо удерживать распберии в сбросе.
#ifdef NDEBUG
  HAL_GPIO_WritePin(BACKUP_EN_GPIO_Port, BACKUP_EN_Pin, GPIO_PIN_SET);
#endif
  Debug::addMsg(Debug::MSG_powerTimeReset);
}
