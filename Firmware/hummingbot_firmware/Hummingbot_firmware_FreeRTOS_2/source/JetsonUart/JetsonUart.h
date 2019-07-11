/*
 * RF24.h
 *
 *  Created on: Apr 28, 2019
 *      Author: jackxu
 */

#ifndef JETSON_UART_H_
#define JETSON_UART_H_

void JU_init(void);
void JU_begin(void);
void JU_prepSync(void);
bool JU_isSynced(void);
bool JU_trySync(void);
void JU_prepXfer(void);
void JU_doXfer(void);

#endif //JETSON_UART_H_
