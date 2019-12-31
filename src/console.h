/*
 * console.h
 *
 * Created: 5/24/2017 
 *  Author: chris
 */ 


#ifndef CONSOLE_H_
#define CONSOLE_H_

void console_init(int port, uint32_t baud);
int console_gets(char *str);
int read_console_input(void);
int console_prompt_int(const char *str, int value, int timeout);




#endif /* INCFILE1_H_ */