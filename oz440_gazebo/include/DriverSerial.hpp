#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
//#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
//#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <unistd.h>
//#include <getopt.h>


int serialport_init(const char* serialport, int baud);
int serialport_writebyte(int fd, uint8_t b);
int serialport_write(int fd, const char* str);
int serialport_read_until(int fd, char* buf, char until);
void flush(int fd);


#endif
