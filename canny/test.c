#include "canny.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

unsigned char buf[256 * 256 * 3];

#define PACK_SIZE 1024 * 4

int main(void)
{
  /* receive image */
  struct{
    int w, h;
  } temp;
  int sd;
  struct sockaddr_in name, cli_name;
//  int sock_opt_val = 1;


  if((sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    printf("socket() error!\n");
    exit(-1);
  }

/*
  if(setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *) &sock_opt_val,
    sizeof(sock_opt_val)) < 0) {
    fprintf(stderr, "Failed to set SO_REUSEADDR on INET socket!\n");
    exit(-1);
  }
*/
  int port = 54000;
  bzero(&name, sizeof(name));
  name.sin_family = AF_INET;
  name.sin_port = htons(port);
  name.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(sd, (struct sockaddr *)&name, sizeof(name)) < 0){
    printf("bind() fails!\n");
    exit(-1);
  }

  int slen = sizeof(cli_name);
  printf("listening on port %d\n", port);
  if(recvfrom(sd, &temp, sizeof(temp), 0, &cli_name, &slen) < 0){
    printf("receive h/w fails!\n");
    exit(-1);
  }
  printf("size received: %d, %d\n", temp.w, temp.h);
  int width = temp.w, height = temp.h;

  //int length = sizeof(char) * width * height * 3;
  int length = 256 * 256 * 3;
  //char *buf = (char *)malloc(length), *ptr = buf;
  unsigned char *ptr = buf;

  int ttt = length;
  while(ttt > 0){
    if(recvfrom(sd, ptr, ttt < PACK_SIZE ? ttt : PACK_SIZE, 0, &cli_name, &slen) < 0){
      printf("receive pixels fails!\n");
      exit(-1);
    }
    ttt -= PACK_SIZE;
    ptr += PACK_SIZE;
  }
printf("data received\n");

  /* image processing */
  Image img;
  I_init(&img, width, height, TYPE_3BYTE_BGR);
printf("after init\n");

  int i, j, index;
/*
  unsigned char a1, a2, a3;
  for(i = 0; i < height; i++){
    for(j = 0; j < width; j++){
      index = (i * width + j) * 3;
      a1 = buf[index];
      a2 = buf[index + 1];
      a3 = buf[index + 2];
      img.data[index] = a1;
      img.data[index + 1] = a2;
      img.data[index + 2] = a3;
    }
  }
printf("after loop\n");
*/

    CEDetector detect;
    C_init(&detect);
printf("after detect init\n");
    detect.lowThreshold = 0.5;
    detect.highThreshold = 1.0;
    detect.sourceImage = &img;
    C_process(&detect);
printf("after process\n");

  /* convert from BGRA to RGB */
  for(i = 0; i < height; i++){
    for(j = 0; j < width; j++){
      index = i * width + j;
      buf[index * 3] = detect.edgesImage->data[index * 4 + 2];
      buf[index * 3 + 1] = detect.edgesImage->data[index * 4 + 1];
      buf[index * 3 + 2] = detect.edgesImage->data[index * 4];
    }
  }
  printf("data processed\n");
  
  /* send result */
  ttt = length;
  ptr = buf;
  while(ttt > 0){
    if(sendto(sd, ptr, ttt < PACK_SIZE ? ttt : PACK_SIZE, 0, (struct sockaddr *)&cli_name, sizeof(cli_name)) < 0){
      printf("sendto() fails!\n");
      exit(-1);
    }
    ttt -= PACK_SIZE;
    ptr += PACK_SIZE;
//    printf("sent\n");
//    printf("sent\n");
//    printf("sent\n");
//    printf("sent\n");
  }

  /* clean up */
  C_deinit(&detect);
  I_deinit(&img);
//    free(buf);
  close(sd);
  return 0;
}



/* vi: set et sw=2 sts=2: */
