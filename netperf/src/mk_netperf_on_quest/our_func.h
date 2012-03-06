

#ifndef __OUR_FUNC_H
#define __OUR_FUNC_H

#include "our_stuff.h"
#include "../../kernel/include/types.h"
#include "sys/select.h"

ssize_t sendmsg (int sockfd, const struct msghdr *msg, int flags);

int snprintf(char *str, size_t size, const char *format, ...);

//int fprintf(FILE *stream, const char *format, ...);

uint32_t htonl(uint32_t hostlong);

uint16_t htons(uint16_t hostshort);

uint32_t ntohl(uint32_t netlong);

uint16_t ntohs(uint16_t netshort);

int fflush(FILE *stream);

int sigemptyset(sigset_t *set);


int sigaction(int signum, const struct sigaction *act,
              struct sigaction *oldact);

int sigaddset(sigset_t *set, int signum);

unsigned int alarm(unsigned int seconds);


pid_t getpid(void);


void perror(const char *s);

void assert(scalar expression);


long double sqrt ( long double x );

int feof(FILE *stream);

void rewind(FILE *stream);

int fcntl(int fd, int cmd, ... /* arg */ );

char *strtok(char *str, const char *delim);

unsigned int sleep(unsigned int seconds);

char *strerror(int errnum);

int gettimeofday(struct timeval *tv, struct timezone *tz);

int settimeofday(const struct timeval *tv, const struct timezone *tz);

double pow(double x, double y);

float powf(float x, float y);

long double powl(long double x, long double y);


long int strtol(const char *nptr, char **endptr, int base);

int getopt(int argc, char ** argv, char *ostr);

int isdigit(int c);

double strtod(const char *nptr, char **endptr);

int sscanf(const char *str, const char *format, ...);

long sysconf(int name);

struct protoent *getprotobyname(const char *name);

int ioctl(int d, int request, ...);

void *dlsym(void *handle, const char *name);

int dlclose(void *handle);

void *dlopen(const char *file, int mode);

char *dlerror(void);

int nftw(const char *dirpath,
        int (*fn) (const char *fpath, const struct stat *sb,
		   int typeflag, struct FTW *ftwbuf),
        int nopenfd, int flags);



int NLMSG_ALIGN(size_t len);


int NLMSG_LENGTH(size_t len);


int NLMSG_SPACE(size_t len);


void *NLMSG_DATA(struct nlmsghdr *nlh);


struct nlmsghdr *NLMSG_NEXT(struct nlmsghdr *nlh, int len);


int NLMSG_OK(struct nlmsghdr *nlh, int len);


int NLMSG_PAYLOAD(struct nlmsghdr *nlh, int len);


int RTA_OK(struct rtattr *rta, int rtabuflen);

void *RTA_DATA(struct rtattr *rta);

unsigned int RTA_PAYLOAD(struct rtattr *rta);

struct rtattr *RTA_NEXT(struct rtattr *rta, unsigned int rtabuflen );

unsigned int RTA_LENGTH(unsigned int length);

unsigned int RTA_SPACE(unsigned int length);

#define RTM_RTA(r)  ((struct rtattr*)(((char*)(r)) + NLMSG_ALIGN(sizeof(struct rtmsg))))

#define RTM_PAYLOAD(n) NLMSG_PAYLOAD(n,sizeof(struct rtmsg))

char *if_indextoname(unsigned ifindex, char *ifname);

int fgetc(FILE *stream);char *fgets(char *s, int size, FILE *stream);

double rint(double x);

off_t lseek(int fd, off_t offset, int whence);

#endif
