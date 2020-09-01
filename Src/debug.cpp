/*
 * Debug.cpp
 *
 *  Created on: 17 рту. 2020 у.
 *      Author: Shcheblykin
 */

#include <debug.h>
#include <limits.h>
#include "main.h"

#define DEBUG_BUF_

uint32_t Debug::debug = 0;
TFifo<Debug::c_bufSize, char> Debug::buf;

const char *Debug::msgString[] = {
    "MSG_i2cWdTimerReset",
    "MSG_i2cActionTimeReset",
    "MSG_i2cErrorCallback",
    "MSG_i2cErrorCounterAf",
    "MSG_HAL_I2C_AbortCpltCallback",
    "MSG_i2cAddrCallback_0x3D",
    "MSG_i2cAddrCallback_0x3E",
    "MSG_i2cAddrCallbackError",
    "MSG_powerExtPwrDownIsLow",
    "MSG_powerTimeReset",
    "MSG_rpiConnectionNo",
    "MSG_rpiReset"
};

uint32_t Debug::msgCnt[Debug::MSG_MAX] = { 0 };

//
int _write(int file, char *ptr, int len) {
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    Debug::buf.push(*ptr++);
  }

  return len;
}

//
Debug::Debug() {
  // TODO Auto-generated constructor stub
  static_assert((sizeof(msgString) / sizeof(msgString[0])) == MSG_MAX,
      "Error msgString size!\n");

  static_assert((sizeof(debug) * CHAR_BIT) >= MSG_MAX,
      "Error size of 'debug' variable\n");
}

//
void Debug::addMsg(msg_t msg) {
  if (msg < MSG_MAX) {
    debug |= (1UL << msg);
    msgCnt[msg]++;
  }
}

//
void Debug::proc() {
  static uint32_t time = 0;

  time++;

  if (debug > 0) {
    printf("Debug message 0x(%.4lX) at time = %ld sek:\n", debug,  time);
  }

  for(uint8_t i = 0; i < MSG_MAX; i++) {
    uint32_t mask = (1UL << i);

    if ((debug & mask) == mask) {
      debug &= ~mask;
      printf("    %s (cnt = %ld)\n", msgString[i], msgCnt[i]);
    }
  }
}

//
void Debug::send() {
  char ch = 0;

  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
      ((ITM->TER & 1UL               ) != 0UL))   /* ITM Port #0 enabled */
  {
    if ((ITM->PORT[0U].u32 != 0UL) && (buf.pop(ch))) {
      ITM->PORT[0U].u8 = ch;
    }
  }
}


