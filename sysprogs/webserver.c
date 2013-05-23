#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#define QUEST // Comment out this line for the webserver to run on Linux

#define BUFSIZE 8096

#define WPORT   8080
#define SERVER_BUFFER_SIZE    (4096 * 5)
#define SBUF_SIZE    3

struct {
  char *ext;
  char *filetype;
} extensions [] = {
  {"gif", "image/gif" },
  {"jpg", "image/jpeg"},
  {"jpeg","image/jpeg"},
  {"png", "image/png" },
  {"zip", "image/zip" },
  {"gz",  "image/gz"  },
  {"tar", "image/tar" },
  {"htm", "text/html" },
  {"html","text/html" },
  {"css", "text/css"  },
  {0,0}
};

typedef struct _server_buffer_t {
  char file_name[256];
  char * payload[SERVER_BUFFER_SIZE];
  unsigned int len;
} server_buffer_t;

static server_buffer_t sbuf[SBUF_SIZE];

int
cache_request (char * fname)
{
  int i, k = 0;
  char tftp_file_name[256];

  sprintf (tftp_file_name, "/boot/%s", fname);
  for (i = 0; i < SBUF_SIZE; i++) {
    if (sbuf[i].len < sbuf[k].len) k = i;
    if (strcmp (tftp_file_name, sbuf[i].file_name) == 0) {
      /* Already cached */
      return i;
    }
  }

  /* Read File */

  /* Stub for Linux */
  int file_fd;
#ifdef QUEST
  if((file_fd = open(tftp_file_name, O_RDONLY)) == -1) { /* open the file for reading */
#else
  if((file_fd = open(fname,O_RDONLY)) == -1) { /* open the file for reading */
#endif
    printf("failed to open file\n");
    return -1;
  }
  sbuf[k].len = read(file_fd, sbuf[k].payload, SERVER_BUFFER_SIZE);
  close (file_fd);
  /* Stub for Linux */

  //sbuf[k].len = read (tftp_file_name, sbuf[k].payload, SERVER_BUFFER_SIZE);

  strcpy (sbuf[k].file_name, tftp_file_name);
 
  return k;
}

void process_http (int fd, int hit)
{
  int j, buflen, len;
  long i, ret;
  char * fstr;
  int cache_index;
  static char buffer[BUFSIZE+1];

  ret = recv (fd, buffer, BUFSIZE, 0); 	/* read Web request */
  if (ret == 0 || ret == -1) {
    printf("failed to read browser request\n");
  }
  if (ret > 0 && ret < BUFSIZE)
    buffer[ret]=0;
  else buffer[0]=0;

  for (i=0;i<ret;i++)
    if(buffer[i] == '\r' || buffer[i] == '\n')
      buffer[i]='*';

  if (strncmp(buffer,"GET ",4) && strncmp(buffer,"get ",4) )
    printf("Only simple GET operation supported\n");

  for (i=4;i<BUFSIZE;i++) {
    if(buffer[i] == ' ') {
      buffer[i] = 0;
      break;
    }
  }

  for (j=0;j<i-1;j++)
    if (buffer[j] == '.' && buffer[j+1] == '.')
      printf("Parent directory (..) path names not supported\n");

  buflen = strlen(buffer);
  fstr = (char *)0;
  for (i=0;extensions[i].ext != 0;i++) {
    len = strlen(extensions[i].ext);
    if ( !strncmp(&buffer[buflen-len], extensions[i].ext, len)) {
      fstr =extensions[i].filetype;
      break;
    }
  }
  if (fstr == 0) printf("file extension type not supported\n");

  cache_index = cache_request (&buffer[5]);

  sprintf (buffer,"HTTP/1.0 200 OK\r\nContent-Type: %s\r\n\r\n", fstr);
  send (fd,buffer,strlen(buffer), 0);

  if(cache_index >= 0) {
    send (fd, sbuf[cache_index].payload, sbuf[cache_index].len, 0);
  }
  close (fd);
}


int main (int argc, char **argv)
{
  int port, listenfd, socketfd, hit;
  socklen_t length;
  static struct sockaddr_in cli_addr;
  static struct sockaddr_in serv_addr;

  printf ("Welcome to Quest HTTP Server!\n");
  printf ("Server Started on Port: %d\n", WPORT);
  printf ("Root Directory Set to: /boot\n");
  /* setup the network socket */
  if ((listenfd = socket (AF_INET, SOCK_STREAM,0)) <0)
    printf("system call socket failed\n");
  port = WPORT;
  if (port < 0 || port >60000)
    printf("Invalid port number (try 1->60000)\n");
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons(port);
  if (bind(listenfd, (struct sockaddr *)&serv_addr,sizeof(serv_addr)) <0)
    printf("system call bind failed\n");
  if (listen(listenfd,64) <0)
    printf("system call listen failed\n");

  for (hit=1; ;hit++) {
    length = sizeof(cli_addr);
    if ((socketfd = accept(listenfd, (struct sockaddr *)&cli_addr, &length)) < 0)
      printf("system call accept failed\n");

    process_http (socketfd,hit); /* never returns */
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
