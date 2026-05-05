#include <stdio.h>
#include <pthread.h>
#include <sys/poll.h>
#include <mqueue.h>
#include "tcp.h"
#include "serial_read.h"
#include <sys/wait.h>

#define MAX_QUEUE_MESS 256

pthread_t t1, t2, t3, t4;
struct pollfd read_fdps;
struct pollfd write_fdps;
struct pollfd connect_fdp;
struct pollfd read_serial_fdp;

FILE *fptr;

mqd_t tcp_to_serial_mq;
mqd_t serial_to_tcp_mq;
struct mq_attr attr;

void *serial_to_tcp_thread(void *arg);
void *serial_port_thread(void *argument);
void *tcp_thread(void *argument);
void *run_model(void *arg);

int main()
{
    attr.mq_flags = 0;
    attr.mq_maxmsg = 1;
    attr.mq_msgsize = MAX_QUEUE_MESS;
    attr.mq_curmsgs = 0;

    mq_unlink("/TCP_TO_SERIAL");

    tcp_to_serial_mq = mq_open("/TCP_TO_SERIAL", O_CREAT | O_RDWR, 0666, &attr);

    if (tcp_to_serial_mq == (mqd_t)-1)
    {
        perror("tcp_to_serial->mq_open");
        return -1;
    }
    fptr = fopen("log.txt", "w");
    pthread_create(&t1, NULL, serial_port_thread, NULL);
    sleep(3);
    pthread_create(&t2, NULL, tcp_thread, NULL);
    pthread_create(&t3, NULL, serial_to_tcp_thread, NULL);
    pthread_create(&t4, NULL, run_model, NULL);

    pthread_join(t1, NULL);
    pthread_detach(t2);
    pthread_detach(t3);

    mq_unlink("/TCP_TO_SERIAL");
    fclose(fptr);
}
/*this thread waits for incoming tcp message and send to serial port*/
void *serial_port_thread(void *argument)
{
    char buffer[256];
    int count_fail = 0;
    int val_write;
    unsigned int priority = 0;
    /*initialize serial port, if failed end hole program*/
begin_serial_thread:
    if (count_fail > 5)
    {
        printf("Connect to %s has failed\n", serial_port_name);
        return NULL;
    }
    serial_fd = connect_port();
    if (serial_fd == PORT_ERROR)
    {
        count_fail++;
        goto begin_serial_thread;
    }
    configureSerialPort(serial_fd, B115200);
    if (acknowledge_port(serial_fd) == PORT_NOT_ACKNOWLEDGE)
    {
        count_fail++;
        printf("Can not recieve acknowledgement\n");
        close(serial_fd);
        goto begin_serial_thread;
    }
    else
    {
        printf("Connect to %s has success\n", serial_port_name);
    }

    read_serial_fdp.fd = serial_fd;
    read_serial_fdp.events = POLLIN;

    /*if receive message from tcp socket, pass to serial port*/
    while (1)
    {
        if (mq_receive(tcp_to_serial_mq, buffer, MAX_QUEUE_MESS, &priority) == -1)
        {
            perror("mq_receive");
            return NULL;
        }
        val_write = write(serial_fd, buffer, strlen(buffer));
        printf("Sent %d character: \"%s\" to %s\n", val_write, buffer, serial_port_name);
        memset(buffer, 0, 256);
    }

    return NULL;
}

/*this thread reads incoming tcp message and send to serial_port_thread*/
void *tcp_thread(void *argument)
{
    int ret = 0;
    char buffer[128];
    /*create and tcp socket server*/
    create_TCP_IPv4_server(&server, PORT, &server_fd, &len);
    connect_fdp.fd = server_fd;
    connect_fdp.events = POLLIN;
    int val_read = 0;
    while (1)
    {
        ret = poll(&connect_fdp, 1, 100);
        if (ret > 0)
        {
            /*wait for 1 tcp socket client*/
            if (accept_client_connection(&server, &server_fd, &client_fd, &len) == ERROR)
            {
                printf("Connection to this client has failed\n");
                return NULL;
            }
            else
            {
                read_fdps.fd = client_fd;
                read_fdps.events = POLLIN;
                write_fdps.fd = client_fd;
                write_fdps.events = POLLOUT;
            }
        }
        else
        {
            continue;
        }

        while (1)
        {
            /*wait for incomming tcp client messages*/
            ret = poll(&read_fdps, 1, 100);
            if (ret == 0)
            {
                continue;
            }
            else if (ret < 0)
            {
                perror("poll of read_fdps");
                break;
            }
            val_read = read(client_fd, buffer, 128);
            if (val_read > 0)
            {
                get_client_information(&client_fd, READ_FROM_TCP, &len);
                printf("%s\n", buffer);
                if (mq_send(tcp_to_serial_mq, buffer, 128, 1) == -1)
                {
                    perror("mq_send");
                    return NULL;
                }
                memset(buffer, 0, 128);
            }
            else if (val_read == 0)
            {
                printf("Client has disconnected\n");
                read_fdps.fd = 0;
                read_fdps.events = 0;
                close(client_fd);
                break;
            }
        }
    }
    return NULL;
}

/*this thread receive serial messages and send to tcp socket*/
void *serial_to_tcp_thread(void *argument)
{
    int ret;
    int val_read, val_write;
    char buffer[128];

    int count = 0;
    while (1)
    {
        /*wait for incomming serial messages*/
        /*if there are incomming messages, send to tcp socket*/
        ret = poll(&read_serial_fdp, 1, 50);
        if (ret > 0 && (read_serial_fdp.revents & POLLIN))
        {
            usleep(2500);
            val_read = read(serial_fd, buffer, 128);
            if (val_read > 0 && (strcmp(buffer, "") != 0))
            {
                if (count < 1000)
                {
                    fprintf(fptr, "Received %s from %s ", buffer, serial_port_name);
                    fprintf(fptr, "Send %d bytes to TCP ", val_read);

                    for (int i = 0; i < 5; i++)
                    {
                        ret = poll(&write_fdps, 1, 100);
                        if (ret == 0)
                        {
                            continue;
                        }
                        else if (ret > 0 && (write_fdps.revents == POLLOUT))
                        {
                            val_write = write(client_fd, buffer, strlen(buffer));
                            fprintf(fptr, "Sent %d bytes to TCP\n", val_write);
                            break;
                        }
                        else if (ret < 0)
                        {
                            perror("write to client_fd");
                            break;
                        }
                    }

                    count++;
                }
                else
                {
                    count = 0;
                    fseek(fptr, 0, SEEK_SET);
                }
            }
            memset(buffer, 0, 128);
        }
    }
}

void *run_model(void *arg)
{
    pid_t pid = fork();

    if (pid == 0)
    {
        execlp("python3", "python3", "yolo_detect.py", NULL);
        perror("execlp failed");
        exit(1);
    }
    else if (pid > 0)
    {
        printf("Started Python YOLO process (PID = %d)\n", pid);
        waitpid(pid, NULL, 0);
    }
    else
    {
        perror("fork failed");
    }

    return NULL;
}
