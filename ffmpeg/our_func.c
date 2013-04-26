#include "our_func.h"
#include "stdio.h"
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//char *optarg;
//int optind = 1, opterr = 1, optopt;

#define unimplemented_funcion_called()					\
  do {									\
    printf("\n\nExiting because %s in %s is unimplemented\n", __FUNCTION__, __FILE__); \
    exit(-1);								\
  } while(0)

//int snprintf(char *str, size_t size, const char *format, ...)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

#define execvp exec

int ftruncate(int a, int b) 
{
  unimplemented_funcion_called();
  return 0;
}

double atof(const char * s)
{
  unimplemented_funcion_called();
  return 0;
}

//int fflush(FILE *stream)
//{
//  return 0;
//}

int dup (int filedes)
{
  unimplemented_funcion_called();
  return 0;
}

int fileno (FILE *fp)
{
  unimplemented_funcion_called();
  return 0;
}

//int stat (const char *pathname, struct stat *buf)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

//int unlink (const char *pathname)
//{
//  return 0;
//}

int chmod (const char *pathname, mode_t mode)
{
  unimplemented_funcion_called();
  return 0;
}

//int sigemptyset(sigset_t *set)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

int sigaction(int signum, const struct sigaction *act,
              struct sigaction *oldact)
{
  unimplemented_funcion_called();
  printf("sigaction called passing signal %d\n", signum);
  return 0;
}


//int sigaddset(sigset_t *set, int signum)
//{
//  unimplemented_funcion_called();
//  return 0;
//}


unsigned int alarm(unsigned int seconds)
{
  unimplemented_funcion_called();
  return 0;
}

//pid_t getpid(void)
//{
//  /*
//   * --!!-- Returning a number to make this work
//   */
//  return 80;
//}



void perror(const char *s)
{
  printf (s);
}


void assert(scalar expression)
{
  if(!expression) {
    printf("Assert failed\n");
    exit(-1);
  }
}


//int feof(FILE *stream)
//{
//  unimplemented_funcion_called();
//  return 0;
//}


void rewind(FILE *stream)
{
  unimplemented_funcion_called();
}

//long double sqrt ( long double x )
//{
//  unimplemented_funcion_called();
//  return 0;
//}

int nftw(const char *dirpath,
        int (*fn) (const char *fpath, const struct stat *sb,
		   int typeflag, struct FTW *ftwbuf),
	 int nopenfd, int flags)
{
  unimplemented_funcion_called();
  return 0;
}

//int fgetc(FILE *stream);char *fgets(char *s, int size, FILE *stream)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

double rint(double x)
{
  unimplemented_funcion_called();
  return 0;
}

ssize_t recvmsg(int sockfd, struct msghdr *msg, int flags)
{
  unimplemented_funcion_called();
  return 0;
}

int fcntl(int fd, int cmd, ... /* arg */ )
{
  unimplemented_funcion_called();
  return 0;
}

char *strtok(char *str, const char *delim)
{
  unimplemented_funcion_called();
  return 0;
}

const char * gai_strerror(int ecode)
{
  unimplemented_funcion_called();
  return 0;
}

//off_t lseek(int fd, off_t offset, int whence)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

unsigned int sleep(unsigned int seconds)
{
  unimplemented_funcion_called();
  return 0;
}

char *strerror(int errnum)
{
  unimplemented_funcion_called();
  return 0;
}

int settimeofday(const struct timeval *tv, const struct timezone *tz)
{
  unimplemented_funcion_called();
  return 0;
}


double pow(double x, double y)
{
  unimplemented_funcion_called();
  return 0;
}


float powf(float x, float y)
{
  unimplemented_funcion_called();
  return 0;
}

long double powl(long double x, long double y)
{
  unimplemented_funcion_called();
  return 0;
}

//long int strtol(const char *nptr, char **endptr, int base)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

#if 0
int getopt(int argc, char **argv, char *ostr)
{
  static char *place = EMSG;    /* option letter processing */
  register char *oli;                   /* option letter list index */
  
  if (!*place) {
    /* update scanning pointer */
      if (optind >= argc || *(place = argv[optind]) != '-' || !*++place) {
        return EOF; 
      }
    if (*place == '-') {
      /* found "--" */
        ++optind;
      place = EMSG ;    /* Added by shiva for Netperf */
        return EOF;
    }
  }
  
  /* option letter okay? */
  if ((optopt = (int)*place++) == (int)':'
      || !(oli = strchr(ostr, optopt))) {
    if (!*place) {
      ++optind;
    }
    printf("illegal option");
    return BADCH;
  }
  if (*++oli != ':') {  
    /* don't need argument */
    optarg = NULL;
    if (!*place)
      ++optind;
  } else {
    /* need an argument */
    if (*place) {
      optarg = place;           /* no white space */
    } else  if (argc <= ++optind) {
      /* no arg */
      place = EMSG;
      printf("option requires an argument");
      return BADCH;
    } else {
      optarg = argv[optind];            /* white space */
    }
    place = EMSG;
    ++optind;
  }
  return optopt;                        /* return option letter */
}
#endif

int isdigit(int c)
{
  unimplemented_funcion_called();
  return 0;
}

//double strtod(const char *nptr, char **endptr)
//{
//  unimplemented_funcion_called();
//  return 0;
//}

int sscanf(const char *str, const char *format, ...)
{
  unimplemented_funcion_called();
  return 0;
}

long sysconf(int name)
{
  unimplemented_funcion_called();
  return 0;
}

static const char *
inet_ntop_v4 (const void *src, char *dst, size_t size)
{
    const char digits[] = "0123456789";
    int i;
    struct in_addr *addr = (struct in_addr *)src;
    u_long a = ntohl(addr->s_addr);
    const char *orig_dst = dst;

    if (size < INET_ADDRSTRLEN) {
      printf("failed in inet_ntop and should set errno but we don't have it so exiting\n");
      exit(-1);
      return NULL;
    }
    for (i = 0; i < 4; ++i) {
	int n = (a >> (24 - i * 8)) & 0xFF;
	int non_zerop = 0;

	if (non_zerop || n / 100 > 0) {
	    *dst++ = digits[n / 100];
	    n %= 100;
	    non_zerop = 1;
	}
	if (non_zerop || n / 10 > 0) {
	    *dst++ = digits[n / 10];
	    n %= 10;
	    non_zerop = 1;
	}
	*dst++ = digits[n];
	if (i != 3)
	    *dst++ = '.';
    }
    *dst++ = '\0';
    return orig_dst;
}

struct protoent *getprotobyname(const char *name)
{
  //printf ("Calling getprotobyname (%s)\n", name);
  //unimplemented_funcion_called();
  return 0;
}


int ioctl(int d, int request, ...)
{
  unimplemented_funcion_called();
  return 0;
}

void *dlsym(void *handle, const char *name)
{
  unimplemented_funcion_called();
  return 0;
}

int dlclose(void *handle)
{
  unimplemented_funcion_called();
  return 0;
}

void *dlopen(const char *file, int mode)
{
  unimplemented_funcion_called();
  return 0;
}

char *dlerror(void)
{
  unimplemented_funcion_called();
  return 0;
}

int NLMSG_ALIGN(size_t len)
{
  unimplemented_funcion_called();
  return 0;
}

int NLMSG_LENGTH(size_t len)
{
  unimplemented_funcion_called();
  return 0;
}

int NLMSG_SPACE(size_t len)
{
  unimplemented_funcion_called();
  return 0;
}

void *NLMSG_DATA(struct nlmsghdr *nlh)
{
  unimplemented_funcion_called();
  return 0;
}

struct nlmsghdr *NLMSG_NEXT(struct nlmsghdr *nlh, int len)
{
  unimplemented_funcion_called();
  return 0;
}

int NLMSG_OK(struct nlmsghdr *nlh, int len)
{
  unimplemented_funcion_called();
  return 0;
}

int NLMSG_PAYLOAD(struct nlmsghdr *nlh, int len)
{
  unimplemented_funcion_called();
  return 0;
}

int RTA_OK(struct rtattr *rta, int rtabuflen)
{
  unimplemented_funcion_called();
  return 0;
}

void *RTA_DATA(struct rtattr *rta)
{
  unimplemented_funcion_called();
  return 0;
}

unsigned int RTA_PAYLOAD(struct rtattr *rta)
{
  unimplemented_funcion_called();
  return 0;
}

struct rtattr *RTA_NEXT(struct rtattr *rta, unsigned int rtabuflen )
{
  unimplemented_funcion_called();
  return 0;
}

unsigned int RTA_LENGTH(unsigned int length)
{
  unimplemented_funcion_called();
  return 0;
}

unsigned int RTA_SPACE(unsigned int length)
{
  unimplemented_funcion_called();
  return 0;
}

char *if_indextoname(unsigned ifindex, char *ifname)
{
  unimplemented_funcion_called();
  return 0;
}

ssize_t
sendmsg (int sockfd, const struct msghdr *msg, int flags)
{
  unimplemented_funcion_called();
  return 0;
}

