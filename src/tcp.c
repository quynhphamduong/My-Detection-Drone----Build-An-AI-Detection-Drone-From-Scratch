#include "tcp.h"

struct sockaddr_in server;

socklen_t len = (socklen_t)sizeof(struct sockaddr_in);

int server_fd;
int client_fd;

char IP[INET_ADDRSTRLEN];
uint16_t port_number;

int create_TCP_IPv4_server(struct sockaddr_in *server_addr, uint16_t port, int *server_fd, socklen_t *len)
{
    *server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (*server_fd < 0)
    {
        *server_fd = 0;
        perror("socket");
        printf("Failed on socket function\n");
        return ERROR;
    }

    server_addr->sin_family = AF_INET;
    server_addr->sin_port = htons(port);
    server_addr->sin_addr.s_addr = INADDR_ANY;

    int opt = 1;
    if (setsockopt(*server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1)
    {
        perror("setsockopt");
        printf("Failed on setsockopt function\n");
        return ERROR;
    }
    if (bind(*server_fd, (struct sockaddr *)server_addr, *len) < 0)
    {
        perror("bind");
        printf("Failed on binding a server\n");
        return ERROR;
    }

    if (listen(*server_fd, 1) < 0)
    {
        perror("listen");
        printf("Failed on establishing a server\n");
        return ERROR;
    }
    printf("Successfully created TCP server\n");
    return SUCCESS;
}

int accept_client_connection(struct sockaddr_in *server_addr, int *server_fd, int *client_fd, socklen_t *len)
{
    struct sockaddr_in client_identity;
    printf("Wating for client connection\n");
    *client_fd = accept(*server_fd, (struct sockaddr *)server_addr, len);
    if ((*client_fd) < 0)
    {
        perror("accept");
        printf("Server has failed to accept client connection\n");
        return ERROR;
    }

    if (getpeername(*client_fd, (struct sockaddr *)&client_identity, len) < 0)
    {
        perror("getpeername");
        printf("Can not get client infomation");
    }
    else
    {
        if (inet_ntop(AF_INET, &client_identity.sin_addr, IP, *len) == NULL)
        {
            perror("inet_ntop");
            printf("Can not get client information\n");
            goto out;
        }
        else
        {
            printf("\nNew client\nIPv4 address:%s\n", IP);
        }
    }

out:
    printf("Successfully connected to TCP client\n\n");
    return 0;
}

int get_client_information(int *client_fd, uint8_t action, socklen_t *len)
{
    struct sockaddr_in client_identity;
    if (getpeername(*client_fd, (struct sockaddr *)&client_identity, len) < 0)
    {
        perror("getpeername");
        printf("Can not get client infomation");
    }
    else
    {
        if (inet_ntop(AF_INET, &client_identity.sin_addr, IP, *len) == NULL)
        {
            perror("inet_ntop");
            printf("Can not get client information\n");
            return ERROR;
        }
        else
        {
        }
    }
    if (action == READ_FROM_TCP)
    {
        printf("From client with IPv4 address %s: ", IP);
    }
    else if (action == WRITE_TO_TCP)
    {
        printf("To client with IPv4 address %s: ", IP);
    }
    return SUCCESS;
}

int read_from_tcp_client(int fd, char *buffer, size_t size)
{
    return read(fd, buffer, size);
}

int write_to_tcp_client(int fd, const char *buffer, size_t size)
{
    return write(fd, buffer, size);
}

int close_tcp_connection(int server_fd, int client_fd)
{
    if (close(client_fd) == -1)
    {
        perror("close client_fd");
        return -1;
    }
    if (close(server_fd) == -1)
    {
        perror("close client_fd");
        return -1;
    }
    return 0;
}