//
// uart_queue.h
//
#ifndef _UART_QUEUE_H_
#define _UART_QUEUE_H_

#include <uart.h>

#define UART_QUEUE_N 2

#define UART_MAX_PORTS 2

#define UART_QUEUE_SIZE 512

#define UART_CARRIAGE_RETURN_TERMINATION 13
#define UART_NEWLINE_TERMINATION 10
#define UART_NULL_TERMINATION 0
#define UART_STAR_TERMINATION 42
#define UART_NO_TERMINATION -1

#define UART_CALLBACK_MODE true

extern enum status_code uart_init_queue(int port, uint32_t baud);

extern enum status_code uart_read_queue(int port, uint8_t *buf, int len);
extern enum status_code uart_read_message_queue(int port, uint8_t *buf, int len);
extern enum status_code uart_read_wait(int port, uint8_t *buf, int len);

extern enum status_code uart_write_queue(int port, uint8_t *buf, int len);

extern enum status_code uart_start_queue(int port);
extern enum status_code uart_stop_queue(int port);
extern void uart_clear_queue(int port);
extern void uart_clear_all_queues(void);
extern enum status_code uart_set_termination(int port, uint8_t term);
extern enum status_code uart_set_echo(int port, uint8_t on);


#endif