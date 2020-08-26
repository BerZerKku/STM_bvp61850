/*
 * Debug.h
 *
 *  Created on: 17 ���. 2020 �.
 *      Author: Shcheblykin
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "fifo.hpp"

/// ������� ������ ������ ��� �������� � ���� �������.
extern "C" int _write(int file, char *ptr, int len);

class Debug {
  /// ������ ������.
  static const int c_bufSize = 256;

  /// ������� ������ ������ � ����� ��������.
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
    MSG_rpiConnectionNo,
    MSG_rpiReset,

  };

  Debug();

  static void addMsg(msg_t msg);
  static void proc();
  static void send();

private:
  static uint32_t debug;
  static TFifo<Debug::c_bufSize, int> buf;
};

#endif /* DEBUG_H_ */
