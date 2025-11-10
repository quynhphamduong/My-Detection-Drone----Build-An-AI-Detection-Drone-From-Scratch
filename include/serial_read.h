#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_HEADER "/dev/ttyUSB"

extern int serial_fd;
extern char serial_port_name[14];

enum
{
    PORT_ERROR = -1,
    PORT_SUCCESS
};

enum Acknowledgement
{
    PORT_NOT_ACKNOWLEDGE = -1,
    PORT_ACKNOWLEDGED,
};

int connect_port();
int configureSerialPort(int fd, int speed);
int readFromSerialPort(int fd, char *buffer, size_t size);
int writeToSerialPort(int fd, const char *buffer, size_t size);
int acknowledge_port(int fd);