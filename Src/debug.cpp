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
TFifo<Debug::c_bufSize, int> Debug::buf;

int _write(int file, char *ptr, int len) {
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    Debug::buf.push(*ptr++);
  }

  return len;
}

Debug::Debug() {
  // TODO Auto-generated constructor stub

}

void Debug::addMsg(msg_t msg) {
  if (msg < sizeof(debug) * CHAR_BIT) {
    debug |= (1UL << msg);
  }
}

void Debug::proc() {
  for(uint8_t i = 0; i < sizeof(debug) * CHAR_BIT; i++) {
    if (debug == 0)
      break;
    if ((debug & (1 << i)) == 0)
      continue;
    debug &= ~(1 << i);

    switch(static_cast<msg_t> (i)) {
      case MSG_i2cWdTimerReset:
        printf("Reset by i2cWdTimer!\n");
        break;
      case MSG_i2cActionTimeReset:
        printf("Reset by i2cActionTime!\n");
        break;
      case MSG_i2cErrorCallback:
        printf("MSG_i2cErrorCallback!\n");
        break;
      case MSG_i2cErrorCounterAf:
        printf("MSG_i2cErrorCounterAf!\n");
        break;
      case MSG_HAL_I2C_AbortCpltCallback:
        printf("HAL_I2C_AbortCpltCallback!\n");
        break;
//      case MSG_i2cAddrCallback_0x3D:
//        printf("i2cAddrCallback_0x3D!\n");
//        break;
//      case MSG_i2cAddrCallback_0x3E:
//        printf("i2cAddrCallback_0x3E!\n");
//        break;
      case MSG_i2cAddrCallbackError:
        printf("MSG_i2cAddrCallbackError\n!");
        break;
      case MSG_rpiConnectionNo:
        printf("rpiConnectionNo!\n");
        break;
      case MSG_rpiReset:
        printf("rpiReset\n");
        break;
    }
  }
}

void Debug::send() {
  int ch = 0;

  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    if ((ITM->PORT[0U].u32 != 0UL) && (buf.pop(ch))) {
      ITM->PORT[0U].u8 = (uint8_t) ch;
    }
  }
}


