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
bool JU_readXfer(jetson_buf_t* readPtr);

#define JETSON_ENABLE_RECEIVER_ONLY_MODE    1
#define JETSON_ENABLE_ECHO_MODE             1
#define JETSPN_ENABLE_SYNC_FIRST            1
#define JETSON_UART_H_TEST_CASE             0
#define JETSON_UART_TEST_STRUCT             0
//testing
#if (JETSON_UART_H_TEST_CASE)
void JU_prepTesfer(void);
void JU_doTesfer(void);
#endif // (JETSON_UART_H_TEST_CASE)

#endif //JETSON_UART_H_
