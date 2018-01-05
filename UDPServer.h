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
#include "MavlinkData.h"
#include "utility.h"

using namespace std;

#define MAXPACKETSIZE 4096

struct UDPTaskData
{
	int sockfd;
	struct sockaddr_in *clientAddress;
	socklen_t *addrLen;
	char *recvBuff;						
};

class UDPServer
{
	public:
	void setup(int port);
	void start();
	void sendStr(string msg);
	void sendBuff(const char *msgBuff, int msgLen);	
	void closeServer();
	void attach(MavlinkData *mavlinkData);
	
	private:
	int sockfd;
	struct sockaddr_in serverAddress;
	struct sockaddr_in clientAddress;
	socklen_t addrLen;
	char recvBuff[ MAXPACKETSIZE ];	
	pthread_t serverThread;
	struct UDPTaskData taskData; 
	MavlinkData *mavlinkData;
	
	static void * Task(void * argv);
	void errorDisplay(const char *s);
};
