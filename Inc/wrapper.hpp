/*
 * wrapper.h
 *
 *  Created on: 10 авг. 2020 г.
 *      Author: Shcheblykin
 *
 *  Обертка для соединения main.c и С++ классов.
 */

#ifndef WRAPPER_HPP_
#define WRAPPER_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/// Вызов после инициализации всей периферии в main().
extern void wrapperMainInit();

/// Вызов из бесконечного цикла main().
extern void wrapperMainLoop();

/// Обратный вызов окончания передачи в USB-VCP.
extern void CdcTransmitCpltFsCallback();

/// Обратный вызов получения байта из USB-VCP.
extern void CdcReceiveFsCallback(uint8_t *buf, uint16_t len) ;

#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_HPP_ */


