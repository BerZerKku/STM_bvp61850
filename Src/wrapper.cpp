/*
 * wrapper.c
 *
 *  Created on: 10 ���. 2020 �.
 *      Author: Shcheblykin
 */


#include "wrapper.hpp"

#include <cassert>
#include <stdio.h>
#include "debug.h"
#include "main.h"
#include "CPP_bvpCommon/bvpCommon.hpp"
#include "CPP_bvpCommon/serial/modbusVp.h"
#include "CPP_bvpCommon/serial/avantpi.h"

using namespace BVP;

extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

static void i2cReset(I2C_HandleTypeDef *hi2c);
static void i2cActionStart(I2C_HandleTypeDef *hi2c, uint32_t size);
static void i2cActionStop();
static void i2cProcessing();
static void i2cWatchDogReset();

static void powerWatchDogOff();
static void powerWatchDog();

static void rpiWatchDog();
static void rpiWatchDogReset();
static void rpiReset();

static void avantPiPoll();
static void modbusPoll();

#define I2C_ACTION_TIME_MIN_MS 20

#define I2C_TIME_RESET_MS 100
#define I2C_MAX_ERROR_COUNTER 5

#define POWER_OFF_TIME_MS 100

#define RPI_REBOOT_TIME_MS 30000
#define RPI_RESET_NO_CONNECT_MS 2000

//
TParam params;
TModbusVp modbus;
TAvantPi avantPi;

template <size_t size, typename type>
struct protocol_t {
  type rxByte;
  type txByte;
  type buf[size];
  BVP::TSerialProtocol *protocol;
};

protocol_t<256, uint8_t> uart6ln;
protocol_t<256, uint8_t> uart1pi;

//template <size_t size, typename type>
//class protocol_t {
//  type rxByte;
//  type txByte;
//  type buf[size];
//  BVP::TSerialProtocol *protocol;
//};

//
enum i2cState_t {
  I2C_STATE_no = 0,
  I2C_STATE_readWait,
  I2C_STATE_readOk,
  I2C_STATE_write,
  I2C_STATE_writeWait,
  I2C_STATE_writeOk,
  //
  I2C_STATE_MAX
};

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

i2cState_t i2cState = I2C_STATE_no; /// ������� ��������� ����������

BvpPkg bvpPkg(BvpPkg::MODE_slave);

/**
 *
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uart1pi.protocol->sendFinished();
    HAL_UART_Receive_IT(&huart1, &uart1pi.rxByte, 1);
  } else if(huart == &huart6){
    uart6ln.protocol->sendFinished();
    HAL_UART_Receive_IT(&huart6, &uart6ln.rxByte, 1);
  }
}

/**
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uart1pi.protocol->push(uart1pi.rxByte);
    HAL_UART_Receive_IT(&huart1, &uart1pi.rxByte, 1);
  } else if (huart == &huart6) {
    uart6ln.protocol->push(uart6ln.rxByte);
    HAL_UART_Receive_IT(&huart6, &uart6ln.rxByte, 1);
  }
}

/**
 *
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // TODO ��������� �������� �� ������ ����������
  if(huart == &huart6) {
    uart6ln.protocol->readError();
    HAL_UART_Receive_IT(&huart6, &uart6ln.rxByte, 1);
  }
}

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint16_t cnt = 0;

  if (htim == &htim6) {
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

    if (!rpiConnection) {
      Debug::addMsg(Debug::MSG_rpiConnectionNo);
    }

    powerWatchDog();
    rpiWatchDog();

    Debug::send();
  } else if (htim == &htim7) {
    modbus.tick();
    avantPi.tick();
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

    i2cState = (i2cState == I2C_STATE_readWait) ?
        I2C_STATE_readOk : I2C_STATE_no;

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

    i2cState = (i2cState == I2C_STATE_writeWait) ?
        I2C_STATE_writeOk : I2C_STATE_no;

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
    i2cActionStop();

    uint32_t error = HAL_I2C_GetError(hi2c);

    Debug::addMsg(Debug::MSG_i2cErrorCallback);
    if (error & HAL_I2C_ERROR_AF) {
      Debug::addMsg(Debug::MSG_i2cErrorCounterAf);
    }

    i2cState = I2C_STATE_no;
    i2cErrorCounter++;
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
    i2cActionStop();

    Debug::addMsg(Debug::MSG_HAL_I2C_AbortCpltCallback);

    i2cState = I2C_STATE_no;
    i2cErrorCounter++;
  }
}


// ����� ����� ������������� ���� ��������� � main().
void wrapperMainInit() {

  i2cWatchDogReset();
  HAL_Delay(10);
  printf("Hello STM32\n");

  params.setValue(BVP::PARAM_vpBtnSAnSbSac, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_vpBtnSA32to01, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_vpBtnSA64to33, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_blkComPrm32to01, BVP::SRC_pi, 0x50505050);
  params.setValue(BVP::PARAM_blkComPrm64to33, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_blkComPrd32to01, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_blkComPrd64to33, BVP::SRC_pi, 0);
  params.setValue(BVP::PARAM_dirControl, BVP::SRC_pi, BVP::DIR_CONTROL_local);
  params.setValue(BVP::PARAM_blkComPrmAll, BVP::SRC_pi, BVP::ON_OFF_on);
  params.setValue(BVP::PARAM_blkComPrmDir, BVP::SRC_pi, 0x55);

  uart6ln.protocol = &modbus;
  uart6ln.protocol->setBuffer(uart6ln.buf, sizeof(uart6ln.buf) / sizeof(uart6ln.buf[0]));
  uart6ln.protocol->setNetAddress(0x0A);
  uart6ln.protocol->setID(SRC_vkey);
  uart6ln.protocol->setTimeTick(100);
  uart6ln.protocol->setup(huart6.Init.BaudRate,
      (huart6.Init.Parity != UART_PARITY_NONE),
      (huart6.Init.StopBits == UART_STOPBITS_2) ? 2 : 1);
  uart6ln.protocol->setEnable(true);

  uart1pi.protocol = &avantPi;
  uart1pi.protocol->setBuffer(uart1pi.buf, sizeof(uart1pi.buf) / sizeof(uart1pi.buf[0]));
  uart1pi.protocol->setNetAddress(0x01);
  uart1pi.protocol->setID(SRC_pi);
  uart1pi.protocol->setTimeTick(100);
  uart1pi.protocol->setup(huart1.Init.BaudRate,
      (huart1.Init.Parity != UART_PARITY_NONE),
      (huart1.Init.StopBits == UART_STOPBITS_2) ? 2 : 1);
  uart1pi.protocol->setEnable(true);
}


// ����� �� ������������ ����� main().
void wrapperMainLoop() {
  HAL_IWDG_Refresh(&hiwdg);

  GPIO_PinState pinstate = GPIO_PIN_RESET;

  pinstate = HAL_GPIO_ReadPin(Sout7_GPIO_Port, Sout7_Pin);
  pinstate = rpiConnection ? pinstate : GPIO_PIN_SET;

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

  i2cProcessing();

  avantPiPoll();
  modbusPoll();
}

// ����� ���������� I2C�
void i2cReset(I2C_HandleTypeDef *hi2c) {
  i2cWatchDogReset();

  i2cState = I2C_STATE_no;
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
  // ����������� ����� �� ��������.
  // TODO ����� �� �������� ����� ������� � ������� ����������! ����� ������ � ����������� �������.
  i2cActionTime = I2C_ACTION_TIME_MIN_MS;
  // 9 = 8 data bits + 1 ack/nack, ����� � ���� ���� �� ������ �� ��� �������.
  i2cActionTime += (size * 9 * 1000)/(hi2c->Init.ClockSpeed);
  i2cAction = true;
}

//
void i2cActionStop() {
  i2cAction = false;
  i2cActionTime = 100;
}

/// ���������� ������ ��� Raspberry.
void rpiWatchDog() {
  if (rpiTimeToReset > 0) {
    rpiTimeToReset--;
  }

  /// ������ ������ �������� � ������� ������ ����� �������.
  if (rpiTimeToReset == 0) {
    rpiReset();
    rpiTimeToReset = RPI_REBOOT_TIME_MS;
  } else {
    HAL_GPIO_WritePin(RASP_RESET_GPIO_Port, RASP_RESET_Pin, GPIO_PIN_SET);
  }
}

//
void rpiWatchDogReset() {
  // FIXME ������� ����� ������ ��� ��������� �������� �������!
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

// TODO ���������� �� ���������� ?!
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
  // ��� ���������� ���������� ���������� ��������� � ������.
#ifdef NDEBUG
  HAL_GPIO_WritePin(BACKUP_EN_GPIO_Port, BACKUP_EN_Pin, GPIO_PIN_SET);
#endif
  Debug::addMsg(Debug::MSG_powerTimeReset);
}

void i2cProcessing() {
  uint8_t *buf = nullptr;
  uint16_t len = 0;
  uint8_t data[DATA_LEN];


  if (i2cState > I2C_STATE_MAX) {
    i2cState = I2C_STATE_MAX;
  }

  switch(i2cState) {
    case I2C_STATE_no: {
      buf = bvpPkg.getRxPkg(len);
      if (HAL_I2C_Slave_Receive_IT(&hi2c2, buf, len) != HAL_BUSY) {
        i2cState = I2C_STATE_readWait;
        i2cActionStart(&hi2c2, len);
      }
    } break;

    case I2C_STATE_readWait: {
    } break;

    case I2C_STATE_readOk: {
      len = 1;
      if (bvpPkg.getDataFromPkg(data, len)) {
        // FIXME ������� ���������� ���������� �����
        // TODO �������� ��������� ��������� ������.
        // TODO  �������� ������������ ������ �� ��������.
        if (bvpPkg.addDataToPkg(data, len)) {
          i2cState = I2C_STATE_write;
        } else {
          i2cState = I2C_STATE_no;
        }
      } else {
        i2cState = I2C_STATE_no;
      }
    } break;

    case I2C_STATE_write: {
      buf = bvpPkg.getRxPkg(len);
      if (HAL_I2C_Slave_Transmit_IT(&hi2c2, buf, len) != HAL_BUSY) {
        i2cState = I2C_STATE_writeWait;
        i2cActionStart(&hi2c2, len);
      }
    } break;

    case  I2C_STATE_writeWait: {
    } break;

    case  I2C_STATE_writeOk: {
      i2cState = I2C_STATE_no;
    } break;

    case I2C_STATE_MAX: {
      assert(false);
    } break;
  }

  HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, GPIO_PIN_RESET);
}

void avantPiPoll() {
  TSerialProtocol *p = uart1pi.protocol;

  if (p->isEnable()) {
    p->read();

    if (p->write()) {
      uint8_t *data = nullptr;
      uint16_t len = p->pop(&data);
      if (len > 0) {
        HAL_UART_Transmit_IT(&huart1, data, len);
      }
    }
  }
}

void modbusPoll() {
  TSerialProtocol *p = uart6ln.protocol;

  if (p->isEnable()) {
    p->read();

    if (p->write()) {
      uint8_t *data = nullptr;
      uint16_t len = p->pop(&data);
      if (len > 0) {
        HAL_UART_Transmit_IT(&huart6, data, len);
      }
    }
  }
}

