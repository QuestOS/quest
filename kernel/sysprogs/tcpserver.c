#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define ECHO_PORT    1234
#define MAX_LINE     255

int main(int argc, char *argv[]) {
  int  list_s;
  int  conn_s;
  short int port;
  struct sockaddr_in servaddr, clientaddr;
  char buffer[MAX_LINE];
  int recv_len;
  socklen_t slen;

  port = ECHO_PORT;

  if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    printf("ECHOSERV: Error creating listening socket.\n");
    exit(1);
  }

  memset ((void *) &servaddr, 0, sizeof(struct sockaddr_in));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(port);

  if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) {
    printf("ECHOSERV: Error calling bind()\n");
    exit(1);
  }

  if ( listen(list_s, 255) < 0 ) {
    printf("ECHOSERV: Error calling listen()\n");
    exit(1);
  }

  while (1) {
    if ((conn_s = accept(list_s, (struct sockaddr *) &clientaddr, &slen)) < 0 ) {
      printf("ECHOSERV: Error calling accept()\n");
      exit(1);
    }

    printf ("Client connected: %s:%d\n", inet_ntoa(clientaddr.sin_addr), ntohs (clientaddr.sin_port));

    if ((recv_len = recv (conn_s, buffer, MAX_LINE - 1, 0)) > 0) {
      buffer[recv_len] = '\0';
      printf ("Received length: %d\n", recv_len);
      printf ("Message received: %s\n", buffer);
    }

    if (recv_len != send (conn_s, buffer, recv_len, 0)) {
      printf ("Less byte sent than received\n");
    }

    if (close (conn_s) == -1) {
      printf ("Socket close failed\n");
    }
  }
}

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
