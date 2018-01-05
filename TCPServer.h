#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>

using namespace std;

#define MAXPACKETSIZE 4096

class TCPServer
{
	public:
	int sockfd, newsockfd, n, pid;
	struct sockaddr_in serverAddress;
	struct sockaddr_in clientAddress;
	pthread_t serverThread;
	char msg[ MAXPACKETSIZE ];

	void setup(int port);
	string start();
	void sendStr(string msg);
	void sendBuff(const char *msgBuff, int msgLen);	
	void closeServer();
	void clean();
	
	private:
	static void * Task(void * argv);
};

