#include "TCPServer.h" 

bool isClientConnected = false;

void* TCPServer::Task(void *arg)
{
	int n;
	int newsockfd=*(int *)arg;
	char msg[MAXPACKETSIZE];
	pthread_detach(pthread_self());
	cout << "enter TCP thread" << endl;	
	while(1)
	{
		n=recv(newsockfd,msg,MAXPACKETSIZE,MSG_DONTWAIT);
		if(n==0)
		{
			cout << "Connection " << newsockfd << " closed" << endl;		
			isClientConnected = false;   
			break;
		}
		msg[n]=0;
		//send(newsockfd,msg,n,0);
		//Message = string(msg);
		usleep(1000);
    }
	return 0;
}

void TCPServer::setup(int port)
{
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	//sockfd=socket(AF_INET,SOCK_DGRAM,0);
 	memset(&serverAddress,0,sizeof(serverAddress));
	serverAddress.sin_family=AF_INET;
	serverAddress.sin_addr.s_addr=htonl(INADDR_ANY);
	serverAddress.sin_port=htons(port);
	bind(sockfd,(struct sockaddr *)&serverAddress, sizeof(serverAddress));
 	listen(sockfd,5);
}

string TCPServer::start()
{
	string str;
	pthread_detach(pthread_self());	
	while(1)
	{
		cout << "Wait for connection" << endl;
		socklen_t sosize  = sizeof(clientAddress);		
		newsockfd = accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
     	if (newsockfd < 0)
		{ 
          	cout << "ERROR on accept" << endl;
			continue;
		}
		str = inet_ntoa(clientAddress.sin_addr);
		cout << "Connection " << newsockfd << " established" << endl;
		//pthread_create(&serverThread,NULL,&Task,(void *)&newsockfd);
		isClientConnected = true;   
		while(1)
		{
			int n;
			usleep(1000);
			n = recv(newsockfd,msg,MAXPACKETSIZE,MSG_DONTWAIT);
			if (n > 0)
			{
				msg[n]=0;
				printf("%s\n", msg);
			}
			else if (n == 0)
			{
				cout << "Connection " << newsockfd << " closed" << endl;		
				isClientConnected = false;   
				break;
			}
			else if (n < 0)/* error*/
			{
				switch (errno)
				{
					case EWOULDBLOCK:
						continue;
					default:
						break;	
				}
			}
		}		
	}
	return str;
}

void TCPServer::sendStr(string msg)
{
	if (isClientConnected)	
		send(newsockfd,msg.c_str(),msg.length(),0);
}

void TCPServer::sendBuff(const char *msgBuff, int msgLen)
{
	if (isClientConnected)	
		send(newsockfd, msgBuff, msgLen,MSG_NOSIGNAL);	
}

void TCPServer::clean()
{
	memset(msg, 0, MAXPACKETSIZE);
}

void TCPServer::closeServer()
{
	close(sockfd);
	close(newsockfd);
} 
