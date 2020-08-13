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

#ifdef __cplusplus
extern "C" {
#endif


/// Вызов после инициализации всей периферии в main().
extern void wrapperMainInit();

/// Вызов из бесконечного цикла main().
extern void wrapperMainLoop();


#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_HPP_ */


