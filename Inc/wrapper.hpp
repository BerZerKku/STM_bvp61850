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

#ifdef __cplusplus
extern "C" {
#endif


/// ����� ����� ������������� ���� ��������� � main().
extern void wrapperMainInit();

/// ����� �� ������������ ����� main().
extern void wrapperMainLoop();


#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_HPP_ */


