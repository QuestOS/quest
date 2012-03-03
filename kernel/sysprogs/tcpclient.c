#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAX_LINE    255 

int main(int argc, char *argv[]) {
  int conn_s;
  struct sockaddr_in servaddr;
  char buffer[MAX_LINE];
  int msglen, recv_len;

  argc = 4;
  argv[0] = "tcpclient";
  argv[1] = "192.168.2.1";
  argv[2] = "1234";
  argv[3] = "helloworld";

  if (argc != 4) {
    printf ("Usage: echoclnt <server ip> <port> <message>\n");
    exit (1);
  }

  if ( (conn_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    printf("ECHOCLNT: Error creating listening socket.\n");
    exit (1);
  }

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons (1234);

  msglen = strlen(argv[3]);

  if (connect(conn_s, (struct sockaddr *) &servaddr, sizeof(struct sockaddr_in)) < 0 ) {
    printf("ECHOCLNT: Error calling connect()\n");
    exit (1);
  }

  if (msglen != send (conn_s, argv[3], msglen, 0)) {
    printf ("Sending failed, message length mismatch\n");
    exit (1);
  } else {
    printf ("Message sent to server, message length: %d\n", msglen);
  }

  if ((recv_len = recv (conn_s, buffer, MAX_LINE - 1, MSG_WAITALL)) > 0) {
    buffer[recv_len] = '\0';
    printf ("Message received: %s\n", buffer);
  } else {
    printf ("Received 0 bytes from server\n");
    exit (0);
  }

  printf("Echo response: %s\n", buffer);

  exit (0);
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
