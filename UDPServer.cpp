#include "UDPServer.h"

#define MAVLINK_PARSE_MSG_MAX_TIME_MS 50

void UDPServer::errorDisplay(const char *s)
{
	perror(s);
	exit(1);
}

void* UDPServer::Task(void *arg)
{
	int recvLen;
	struct UDPTaskData taskData =*(struct UDPTaskData *)arg;
	pthread_detach(pthread_self());
	cout << "enter UDP thread";
	while(1)
	{
		recvLen = recvfrom(taskData.sockfd, taskData.recvBuff, MAXPACKETSIZE, 0, 
			(struct sockaddr*)taskData.clientAddress, taskData.addrLen); 
		if (recvLen > 0) 
		{ 
			//parse mavlink msg
			printf("Received packet from %s:%d\n", inet_ntoa(taskData.clientAddress->sin_addr), 
				ntohs(taskData.clientAddress->sin_port));
			taskData.recvBuff[recvLen] = 0;
			printf("Data: %s\n" , taskData.recvBuff);
		}
		usleep(1000);
	}
	return 0;
}

void UDPServer::setup(int port)
{
	sockfd=socket(AF_INET,SOCK_DGRAM,0);
	if (sockfd == -1)
		errorDisplay("Socket error");

	memset(&serverAddress,0,sizeof(serverAddress));
	
	serverAddress.sin_family=AF_INET;
	serverAddress.sin_addr.s_addr=htonl(INADDR_ANY);
	serverAddress.sin_port=htons(port);
	
	if (bind(sockfd,(struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
		errorDisplay("Bind error");	

	clientAddress.sin_addr.s_addr = 0;

	addrLen = sizeof(clientAddress); /* length of addresses */		
}

void UDPServer::start()
{
	int recvLen;
	taskData.clientAddress = &clientAddress;
	taskData.addrLen = &addrLen;
	taskData.recvBuff = recvBuff;
	taskData.sockfd = sockfd;
	while(1)
	{
		recvLen = recvfrom(taskData.sockfd, taskData.recvBuff, MAXPACKETSIZE, 0, 
			(struct sockaddr*)taskData.clientAddress, taskData.addrLen); 
		if (recvLen > 0) 
		{ 
			//parse mavlink msg
			mavlink_message_t msg;
			mavlink_status_t status;
			uint32_t tstart_us = Utility::millis();
			for (uint16_t i=0; i<recvLen; i++)
			{
				uint8_t c = taskData.recvBuff[i];
				bool parsed_packet = false;

        		// Try to get a new message
				if (mavlinkData)
				{
					if (mavlinkData->my_mavlink_parse_char(c, &msg, &status)) 
					{					
						mavlinkData->handleMessage(&msg);		
						parsed_packet = true;
					}
				}
				if (parsed_packet || i % 100 == 0) 
				{
            		// make sure we don't spend too much time parsing mavlink messages
					if (Utility::millis() - tstart_us > MAVLINK_PARSE_MSG_MAX_TIME_MS) {
						break;
					}
				}
			}
			//printf("Received packet from %s:%d\n", inet_ntoa(taskData.clientAddress->sin_addr), 
			//	ntohs(taskData.clientAddress->sin_port));
			//taskData.recvBuff[recvLen] = 0;
			//printf("Data: %s\n" , taskData.recvBuff);
		}
		usleep(1000);
	}	
	//pthread_create(&serverThread,NULL,&Task,(void *)&taskData);
}

void UDPServer::sendStr(string msg)
{
	if (clientAddress.sin_addr.s_addr)	
		sendto(sockfd, msg.c_str(), msg.length(), 0, (struct sockaddr*) &clientAddress, addrLen);
}

void UDPServer::sendBuff(const char *msgBuff, int msgLen)
{
	if (clientAddress.sin_addr.s_addr)
		sendto(sockfd, msgBuff, msgLen, 0, (struct sockaddr*) &clientAddress, addrLen);
}

void UDPServer::closeServer()
{
	close(sockfd);
} 

void UDPServer::attach(MavlinkData *mavlinkData)
{
	this->mavlinkData = mavlinkData;
}