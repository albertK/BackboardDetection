#include <stdio.h>
#include <string.h>    //strlen
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <cstdlib>

int main(int argc , char *argv[])
{    
    //建立一個socket
    //AF_INET代表IPv4
    //SOCK_STREAM代表TCP
    //IPPROTO_IP代表自動決定要使用那一network layer的ˊprotocol
    int socket_desc = socket(AF_INET , SOCK_STREAM , IPPROTO_IP);
    if (socket_desc == -1)
    {
	printf("Could not create socket\n");
	return 1;
    }
    
    //socket連線資訊
    //AF_INET代表是IPv4
    //INADDR_ANY代表任何IP都可以接受(因為是server的緣故)
    //htons( 8888 )代表這個socket使用port 8888
    sockaddr_in server;
    server.sin_family = AF_INET;
    //server.sin_addr.s_addr = inet_addr("74.125.235.20");
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 27015 );
    
    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
	puts("bind failed");
	return 1;
    }
    puts("bind done");
    
    int flag = 0;
    for(float i = 1; i < 3.5; ++i)
    {
	//Listen
	//聽看看有沒有其他socket要連線進來
	//最多接受1個連線
	listen(socket_desc , 1);
	
	//Accept an incoming connection
	puts("Waiting for incoming connections...");
	int c = sizeof(struct sockaddr_in);
	//block住直到有client連過來，然後本server socket接受連線
	//連線進來後開一個新的socket new_socket來進行處理，本來的socket繼續listen
	//連線進來的socket資訊存到client中
	sockaddr_in client;
	int new_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
	if (new_socket<0)
	{
	    perror("accept failed");
	    return 1;
	}
	puts("Connection accepted");
	
	/*
	char recv_msg[2];
	unsigned char send_buff[2];
	float number = 2.5;
	while(1)
	{
	    recv(new_socket, recv_msg, 2, 0);
	    if(recv_msg[0] == 'c')
		break;
	    else
	    {
		if(number >= 0)
		    send_buff[0] = 0;
		else
		    send_buff[0] = 1;
		send_buff[1] = abs((int)number);
		
		send(new_socket, &send_buff, 2, 0);
		printf("I send (%d, %d)\n", send_buff[0], send_buff[1]);
	    }
	}
	*/
	
	
	char recv_msg[2];
	recv(new_socket, recv_msg, 2, 0);
	printf("I receive: %c\n", recv_msg[0]);
	if(recv_msg[0] == 'c')
	    break;
	else
	{
	    float number = i*30.0;
	    unsigned char send_buff[2];
	    if(number >= 0)
		send_buff[0] = 0;
	    else
		send_buff[0] = 1;
	    send_buff[1] = abs((int)number);
	    
	    send(new_socket, &send_buff, 2, 0);
	    printf("I send (%d, %d)\n", send_buff[0], send_buff[1]);
	}
	
	flag = (flag+1)%3;
	
	//關閉連線
	printf("Close socket...\n");
	close(new_socket);
    }
    
    ////////////////////////
    //Listen
    //聽看看有沒有其他socket要連線進來
    //最多接受1個連線
    listen(socket_desc , 1);
    
    //Accept an incoming connection
    puts("Waiting for incoming connections...");
    int c = sizeof(struct sockaddr_in);
    //block住直到有client連過來，然後本server socket接受連線
    //連線進來後開一個新的socket new_socket來進行處理，本來的socket繼續listen
    //連線進來的socket資訊存到client中
    sockaddr_in client;
    int new_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
    if (new_socket<0)
    {
	perror("accept failed");
	return 1;
    }
    puts("Connection accepted");
    
    char recv_msg[2];
    recv(new_socket, recv_msg, 2, 0);
    printf("I receive: %c\n", recv_msg[0]);
    float number = 0;
    unsigned char send_buff[2];
    if(number >= 0)
	send_buff[0] = 0;
    else
	send_buff[0] = 1;
    send_buff[1] = abs((int)number);
    
    send(new_socket, &send_buff, 2, 0);
    printf("I send (%d, %d)\n", send_buff[0], send_buff[1]);
    
    //關閉連線
    printf("Close socket...\n");
    close(new_socket);
    ////////////////////////
    
    
    printf("Close program...\n");
    close(socket_desc);
    return 0;
}