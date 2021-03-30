#include <stdio.h> //printf
#include <string.h>    //strlen
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <string>
#include <iostream>	

int main(int argc , char *argv[])
{
    int sock;
    struct sockaddr_in server;
    std::string message,servo_id,pos;
    char server_reply[2000];

    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( 8888 );

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }

    puts("Connected\n");

    //keep communicating with server
    while(1)
    {
        printf("Enter servo : ");
        getline (std::cin, servo_id);
        printf("Enter position : ");
        getline (std::cin, pos);
        
        message = "#" + servo_id + " P" + pos + " T2000" + '\r';

        //Send some data
        if( sendto(sock , message.c_str() , strlen(message.c_str()) , 0, (const struct sockaddr *)&server,sizeof(server)) < 0)
        {
            puts("Send failed");
            return 1;
        }

        //Receive a reply from the server
        if( recv(sock , server_reply , 2000 , 0) < 0)
        {
            puts("recv failed");
            break;
        }

        puts("Server reply :");
        puts(server_reply);
    }

    shutdown(sock,SHUT_RDWR);

    return 0;
}
