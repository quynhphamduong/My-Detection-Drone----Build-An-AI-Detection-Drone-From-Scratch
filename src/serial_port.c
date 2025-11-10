#include "serial_read.h"

int serial_fd;
char serial_port_name[14];

int connect_port()
{
    int fd = 0;
    for (int i = 0; i < 10; i++)
    {
        sprintf(serial_port_name, "%s%d", SERIAL_HEADER, i);
        printf("Binding port serial number %d\n", i);
        fd = open(serial_port_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd != -1)
        {
            printf("Program has connected to %s\n", serial_port_name);
            return fd;
        }
    }
    return PORT_ERROR;
}

int configureSerialPort(int fd, int speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no
                                                // canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // read doesn't block
    tty.c_cc[VTIME] = 10;                       // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        return PORT_ERROR;
    }
    return PORT_SUCCESS;
}

int readFromSerialPort(int fd, char *buffer, size_t size)
{
    return read(fd, buffer, size);
}

// Function to write data to the serial port
int writeToSerialPort(int fd, const char *buffer, size_t size)
{
    return write(fd, buffer, size);
}

int acknowledge_port(int fd)
{
    char temp_buffer[3];
    memset(temp_buffer, 0, 3);
    int val_read = 0;
    sprintf(temp_buffer, "C\n");
    write(fd, temp_buffer, 2);
    val_read = read(fd, temp_buffer, 1);
    if (val_read <= 0)
    {
        return PORT_NOT_ACKNOWLEDGE;
    }
    if (temp_buffer[0] == 'A')
    {
        return PORT_ACKNOWLEDGED;
    }
    return PORT_NOT_ACKNOWLEDGE;
}