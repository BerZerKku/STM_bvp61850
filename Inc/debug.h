/*
 * Debug.h
 *
 *  Created on: 17 авг. 2020 г.
 *      Author: Shcheblykin
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "fifo.hpp"

/// Функция записи данных для передачи в порт отладки.
extern "C" int _write(int file, char *ptr, int len);

class Debug {
  /// Размер буфера.
  static const uint16_t c_bufSize = 512;

  /// Функция записи данных в буфер передачи.
  friend int _write(int file, char *ptr, int len);

public:
  enum msg_t {
    MSG_i2cWdTimerReset = 0,
    MSG_i2cActionTimeReset,
    MSG_i2cErrorCallback,
    MSG_i2cErrorCounterAf,
    MSG_HAL_I2C_AbortCpltCallback,
    MSG_i2cAddrCallback_0x3D,
    MSG_i2cAddrCallback_0x3E,
    MSG_i2cAddrCallbackError,
    MSG_powerExtPwrDownIsLow,
    MSG_powerTimeReset,
    MSG_rpiConnectionNo,
    MSG_rpiReset,
    //
    MSG_MAX
  };

  Debug();

  static void addMsg(msg_t msg);
  static void proc();
  static void send();

private:
  static TFifo<Debug::c_bufSize, char> buf;

  static uint32_t debug;

  static uint32_t msgCnt[MSG_MAX];
  static const char *msgString[];
};

#endif /* DEBUG_H_ */
