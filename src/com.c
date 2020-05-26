/*
 * com.c: 
 * UART serial command interface
 * Embedded Ocean Systems (EOS), 2019
 *
 *----------------------------------------------------------------
*/

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>

#include "wispr.h"
#include "com.h"
#include "uart_queue.h"
#include "wdt.h"
#include <delay.h>

static int verbose_level = 0;

int com_init(int port, uint32_t baud)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port > 1) ) {
		return(0);
	}
	stat = uart_init_queue(port, baud);
	if(stat != STATUS_OK ) {
		return(0);
	}
	
	uart_set_termination(port, UART_STAR_TERMINATION);
	uart_clear_queue(port);
	uart_start_queue(port);

	return(1);
}

void com_stop(int port)
{
	if( (port < 0) || (port > 1) ) {
		return;
	}
	uart_stop_queue(port);
}

//---------------------------------------------------------------------
/*
* Read the next message from the serial port.
* The input message buffer (msg) must be at least COM_MESSAGE_SIZE bytes 
* In canonical mode read() will only return a full line of input. 
* A line is by default terminated by a NL (ASCII LF), an end of file, 
* or an end of line character. 
* A CR will not terminate a line with the default settings.
* This read function will strip the prefix and suffix char from the read message.
* Example;
*  nrd = 37 bytes
*  tmp = $GPS,1420070460,19.000000,19.000000*n
*  msg = GPS,1420070460,19.000000,19.000000
*        0123456789012345678901234567890123456
*/
int com_read_msg (int port, char *msg, int timeout)
{
  int nrd = 0;
  char tmp[COM_MAX_MESSAGE_SIZE];  // input buffer
  int len;
  char *head, *tail;

  // clear message buffers
  memset(tmp, 0, COM_MAX_MESSAGE_SIZE);  
  len = strlen(msg);
  if(len > COM_MAX_MESSAGE_SIZE) len = COM_MAX_MESSAGE_SIZE;
  memset(msg, 0, len);  
  
  // read the message from the port
  enum status_code stat;
  while(1) {
	  stat =  uart_read_message_queue(port, (uint8_t *)tmp, COM_MAX_MESSAGE_SIZE);
	  if(stat == STATUS_OK || timeout <= 0 ) break;
	  wdt_restart(WDT);
	  delay_ms(1);
	  timeout--;
  }
  
  // read the message with no timeout
  //stat =  uart_read_message_queue(port, (uint8_t *)tmp, COM_MAX_MESSAGE_SIZE);
  if(stat != STATUS_OK) return(0);
  
  nrd = strlen(tmp);
  
  // otherwise something was read, so
  // check if it's a valid message 
  if(nrd > 0) {
    // find start, end, and size of the msg in buffer
    head = strchr(tmp, COM_MESSAGE_PREFIX);	// find start of msg
    tail = strchr(tmp, COM_MESSAGE_SUFFIX);	// find end of msg
    len = (int)(tail - head - 1);	// size of message
    if ((len > 0) && (len < COM_MAX_MESSAGE_SIZE)) {
       // copy message into msg buffer, skipping the prefix char
       strncpy (msg, head + 1, len);
       // terminate with NULL, overwriting the suffix char
       msg[len] = 0x00;
    }
  }
  
  // return the length of the message, 
  // which will be 0 if it's not valid
  nrd = strlen(msg);
  if((verbose_level >= 2) && (nrd > 0)) { 
	printf( "com_read_msg: %s, %d bytes\r\n", msg, nrd);
  }
  return (nrd);

}

//---------------------------------------------------------------------
int com_write_msg (int port, char *msg)
{
  int len, nwrt;
  char obuf[COM_MAX_MESSAGE_SIZE];  // output buffer

  len = strlen(msg);  // length of the message

  // check to make sure message is not too long to fix in output buffer
  if(len > (COM_MAX_MESSAGE_SIZE - 4)) len = COM_MAX_MESSAGE_SIZE - 4;

  // copy msg into transmit buffer
  strncpy (obuf + 1, msg, len);  

  // Add prefix, suffix, and a NL to the end and null terminate
  obuf[0] = COM_MESSAGE_PREFIX;  // add prefix
  obuf[len + 1] = COM_MESSAGE_SUFFIX; // add suffix
  obuf[len + 2] = 0x0a;	// <LF> newline 
  obuf[len + 3] = 0x00;	// null terminate the string

  len = strlen(obuf);
  enum status_code stat;
  stat = uart_write_queue(port, (uint8_t *)obuf, len);
  if (stat != STATUS_OK) {
    printf( "com_write_msg: error %d, %s\r\n", stat, obuf);
    return 0;
  }
  
  if(verbose_level > 2) printf( "com_write_msg: %d, %s\r\n", nwrt, obuf);

  return (nwrt);
}

//---------------------------------------------------------------------
// Add your own message parsing here
//
int com_parse_msg (wispr_com_msg_t *msg, char *buf, int len)
{
  char args[COM_MAX_MESSAGE_SIZE-4];

  if (verbose_level) {
    printf("com_parse_msg: %s\r\n", buf);
  }

  msg->type = COM_UNKNOWN;
  
  // Add user commands here

  if (strncmp (buf, "EXI", 3) == 0) {  // Exit command  
    msg->type = COM_EXIT;
  }
  if (strncmp (buf, "RUN", 3) == 0) {  // Run command
    msg->type = COM_RUN;
  }
  if (strncmp (buf, "PAU", 3) == 0) {  // Pause command
    msg->type = COM_PAUSE;
  }
  if (strncmp (buf, "RST", 3) == 0) {  // Reset command
    msg->type = COM_RESET;
  }
  if (strncmp (buf, "SLP", 3) == 0) {  // Sleep command
    msg->type = COM_SLEEP;
  }
  if (strncmp (buf, "STA", 3) == 0) { // Status command
	  msg->type = COM_STATUS;
  }
  if (strncmp (buf, "SET", 3) == 0) { // Set command
	  msg->type = COM_SET;
  }

  // GPS message
  if (strncmp (buf, "GPS", 3) == 0) {
    msg->type = COM_GPS;
    strcpy (args, &buf[4]);  // copy args
    sscanf (args, "%lu,%f,%f", &msg->sec, &msg->lat, &msg->lon);
    if(verbose_level) {
      printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", msg->sec, msg->lat, msg->lon);
    }
  }

  // set time
  if (strncmp (buf, "TME", 3) == 0) { 
     msg->type = COM_TIME;
     strcpy (args, &buf[4]);  // copy args
     sscanf (args, "%lu", &msg->sec);
  }

  // set gain
  if (strncmp (buf, "NGN", 3) == 0) {
	  msg->type = COM_GAIN;
	  strcpy (args, &buf[4]);  // copy args
	  sscanf (args, "%d", &msg->gain);
  }

  // report SD card memory usage
  if (strncmp (buf, "SDF", 3) == 0) {
	  msg->type = COM_SDF;
  }

  if (verbose_level > 2) {
    printf("com_parse_msg: type=%d\r\n", msg->type);
  }

  return (msg->type);
}


int com_request_gps(wispr_com_msg_t *msg, uint16_t timeout)
{
	char buf[COM_MAX_MESSAGE_SIZE]; 
	
	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set DS3231 & RTC.\r\n");
	msg->lat = 0.0; 
	msg->lon = 0.0;
	
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
	
	int nrd = com_read_msg (BOARD_COM_PORT, buf, timeout);
	
	if(nrd > 0) {
		com_parse_msg(msg, buf, nrd);
		printf("lat=%f, lon=%f\n\r", msg->lat, msg->lon); //debug Remove this
	}
	
	return(nrd);
}

int com_request_gain(wispr_com_msg_t *msg, uint16_t timeout)
{
	uint8_t new_gain;
	char buf[COM_MAX_MESSAGE_SIZE];
	
	//HM added to check if a gain change is requested
	com_write_msg(BOARD_COM_PORT, "NGN");

	printf("Type at com0 $NGN,1*\r\n");//HM For debug. Remove this

	int nrd = com_read_msg (BOARD_COM_PORT, buf, timeout);

	new_gain = ADC_DEFAULT_GAIN;
	
	if(nrd > 0) {
		com_parse_msg(msg, buf, nrd);
		//printf("%d\n\r",com_msg.gain);//  HM debug
		//printf("new gain = %d\n\r", com_msg.gain);//HM debug
		if((msg->gain < 4) && (msg->gain >= 0)) {
			new_gain = msg->gain;//HM Gain update is ready. config.settings[0] will be updated in initialize_config()
		}
	}

	// set the global config with the new gain
	msg->gain = new_gain;
	
	return(nrd);

}

