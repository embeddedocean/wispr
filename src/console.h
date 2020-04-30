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
float console_prompt_f32(const char *prompt, float default_value, int timeout);

int console_input(char *str, int size, int timeout);
uint32_t console_prompt_uint32(const char *str, uint32_t value, int timeout);
uint16_t console_prompt_uint16(const char *str, uint16_t value, int timeout);
uint8_t console_prompt_uint8(const char *str, uint8_t value, int timeout);




#endif /* INCFILE1_H_ */