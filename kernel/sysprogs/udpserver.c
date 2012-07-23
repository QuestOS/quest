#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define BSIZE  255
#define SERVER_PORT  1234

int
main (int argc, char * argv[])
{
  int sock;
  struct sockaddr_in echoserver;
  struct sockaddr_in echoclient;
  char buffer[BSIZE];
  int clientlen;
  int recvlen;

  sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  
  if (sock < 0) {
    printf ("Cannot create socket\n");
    exit (1);
  }

  memset ((void *) &echoserver, 0, sizeof (echoserver));

  echoserver.sin_family = AF_INET;
  echoserver.sin_addr.s_addr = htonl (INADDR_ANY);
  echoserver.sin_port = htons (SERVER_PORT);

  if (bind (sock, (struct sockaddr *) &echoserver, sizeof (echoserver)) < 0) {
    printf ("Cannot bind socket\n");
    exit (1);
  }

  clientlen = sizeof (echoclient);

  while (1) {
    recvlen = recvfrom (sock, buffer, BSIZE, 0, (struct sockaddr *) &echoclient, &clientlen);
    if (recvlen < 0) {
      printf ("echoserver receive failed\n");
      break;
    }
    buffer[recvlen] = '\0';

    printf ("Client connected: %s\n", inet_ntoa(echoclient.sin_addr));
    printf ("Message received: %s\n", buffer);

    if (sendto (sock, buffer, recvlen, 0,
                (struct sockaddr *) &echoclient,
                sizeof (echoclient)) != recvlen) {
      printf ("Sending and Receiving length doesn't match\n");
      break;
    } else {
      printf ("Message echoed back to: %s:%d\n", inet_ntoa(echoclient.sin_addr), ntohs (echoclient.sin_port));
    }
  }
}

