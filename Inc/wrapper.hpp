/*
 * wrapper.h
 *
 *  Created on: 10 ���. 2020 �.
 *      Author: Shcheblykin
 *
 *  ������� ��� ���������� main.c � �++ �������.
 */

#ifndef WRAPPER_HPP_
#define WRAPPER_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/// ����� ����� ������������� ���� ��������� � main().
extern void wrapperMainInit();

/// ����� �� ������������ ����� main().
extern void wrapperMainLoop();

/// �������� ����� ��������� �������� � USB-VCP.
extern void CdcTransmitCpltFsCallback();

/// �������� ����� ��������� ����� �� USB-VCP.
extern void CdcReceiveFsCallback(uint8_t *buf, uint16_t len) ;

#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_HPP_ */


