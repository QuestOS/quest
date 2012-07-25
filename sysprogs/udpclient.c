#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define BSIZE  255

int
main (int argc, char * argv[])
{
  int sock;
  struct sockaddr_in echoserver;
  struct sockaddr_in echoclient;
  char buffer[BSIZE];
  int clientlen, msglen;
  int recvlen;

  char * arg1 = "192.168.2.1";
  char * arg2 = "1234";
  char * arg3 = "hello";

  argv[1] = arg1;
  argv[2] = arg2;
  argv[3] = arg3;
  argc = 4;

  if (argc != 4) {
    printf ("Usage: echoclient <server ip> <port> <message>\n");
    exit (1);
  }

  sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  
  if (sock < 0) {
    printf ("Cannot create socket\n");
    exit (1);
  }

  memset ((void *) &echoserver, 0, sizeof (echoserver));

  echoserver.sin_family = AF_INET;
  echoserver.sin_addr.s_addr = inet_addr(argv[1]);
  //echoserver.sin_port = htons (atoi (argv[2]));
  echoserver.sin_port = htons ((uint16_t)1234);

  msglen = strlen(argv[3]);

  if (sendto(sock, argv[3], msglen, 0,
      (struct sockaddr *) &echoserver,
      sizeof(echoserver)) != msglen) {
    printf ("Sending failed, message length mismatch\n");
    goto DIE;
  }

  printf ("Message sent to server: %s\n", argv[3]);

  clientlen = sizeof(echoclient);

  if ((recvlen = recvfrom (sock, buffer, BSIZE, 0,
                           (struct sockaddr *) &echoclient,
                           &clientlen)) != msglen) {
    printf ("Received less than sent\n");
    goto DIE;
  }

  if (echoserver.sin_addr.s_addr != echoclient.sin_addr.s_addr) {
    printf ("Server IP doesn't match\n");
    goto DIE;
  }

  buffer[recvlen] = '\0';
  printf ("Message received from server: %s\n", buffer);
  close (sock);
  exit (0);

DIE:
  close (sock);
  exit (1);
}

