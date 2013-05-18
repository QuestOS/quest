#include "errno.h"
#include "sys/types.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sys/time.h"
#include "sys/times.h"
#include "stdint.h"
#include "netdb.h"
#include "sys/signal.h"
#include "sys/stat.h"

#ifndef _OUR_STUFF_H_
#define _OUR_STUFF_H_

#define fprintf(f,fmt,...)  printf(fmt,##__VA_ARGS__)

//#define BUFSIZ 8192
#define MAX_PATH 256
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

#define _SS_ALIGNSIZE   (sizeof(sint64))
#define _SS_MAXSIZE     128U

//#define _SC_ARG_MAX        1
//#define _SC_CHILD_MAX      2
//#define _SC_CLOCKS_PER_SEC 3
//#define _SC_CLK_TCK        3
//#define _SC_NGROUPS_MAX    4
//#define _SC_OPEN_MAX       5
//#define _SC_JOB_CONTROL    6
//#define _SC_SAVED_IDS      7
//#define _SC_VERSION        8
//#define _SC_STREAM_MAX     9
//#define _SC_TZNAME_MAX    10
//#define _SC_PAGESIZE      11
//#define _SC_PAGE_SIZE     _SC_PAGESIZE


#define IP_HDRINCL      3
#define IP_OPTIONS      4
#define IP_ROUTER_ALERT 5
#define IP_RECVOPTS     6
#define IP_RETOPTS      7
#define IP_PKTINFO      8
#define IP_PKTOPTIONS   9
#define IP_MTU_DISCOVER 10
#define IP_RECVERR      11
#define IP_RECVTTL      12
#define IP_RECVTOS      13
#define IP_MTU          14
#define IP_FREEBIND     15
#define IP_IPSEC_POLICY 16
#define IP_XFRM_POLICY  17
#define IP_PASSSEC      18
#define IP_TRANSPARENT  19

#define NETLINK_ROUTE           0       /* Routing/device hook                          */
#define NETLINK_SKIP            1       /* Reserved for ENskip                          */
#define NETLINK_USERSOCK        2       /* Reserved for user mode socket protocols      */
#define NETLINK_FIREWALL        3       /* Firewalling hook                             */
#define NETLINK_TCPDIAG         4       /* TCP socket monitoring                        */
#define NETLINK_NFLOG           5       /* netfilter/iptables ULOG */
#define NETLINK_ARPD            8
#define NETLINK_ROUTE6          11      /* af_inet6 route comm channel */
#define NETLINK_IP6_FW          13
#define NETLINK_DNRTMSG         14      /* DECnet routing messages */
#define NETLINK_TAPBASE         16      /* 16 to 31 are ethertap */

#define RTA_UNSPEC            0
#define RTA_DST               1
#define RTA_SRC               2
#define RTA_IIF               3
#define RTA_OIF               4
#define RTA_GATEWAY           5
#define RTA_PRIORITY          6
#define RTA_PREFSRC           7
#define RTA_METRICS           8
#define RTA_MULTIPATH         9
#define RTA_PROTOINFO         10
#define RTA_FLOW              11
#define RTA_CACHEINFO         12
#define RTA_SESSION           13

#define SIOCETHTOOL 0x8946

#define RTLD_LAZY 1

# define INET6_ADDRSTRLEN 46

# define INET_ADDRSTRLEN 16

typedef long time_t;
typedef long suseconds_t;
typedef int scalar;
typedef int __kernel_size_t;
typedef unsigned short ushort;
typedef unsigned short u_short;
typedef unsigned long u_long;
typedef socklen_t netperf_socklen_t;
typedef char* caddr_t;

//typedef unsigned long ino_t;
//typedef unsigned short mode_t;
//typedef short nlink_t;
//typedef char gid_t;
typedef short dev_t;
typedef long off_t;
typedef	int blksize_t;
typedef long blkcnt_t;
typedef unsigned int __u32;
typedef unsigned short __u16;




# define SEEK_SET	0	/* Seek from beginning of file.  */
# define SEEK_CUR	1	/* Seek from current position.  */
# define SEEK_END	2	/* Seek from end of file.  */

struct msghdr {
    void    *   msg_name;   /* Socket name          */
    int     msg_namelen;    /* Length of name       */
    struct iovec *  msg_iov;    /* Data blocks          */
    __kernel_size_t msg_iovlen; /* Number of blocks     */
    void    *   msg_control;    /* Per protocol magic (eg BSD file descriptor
								   passing) */
    __kernel_size_t msg_controllen; /* Length of cmsg list */
    unsigned    msg_flags;
};

int errno;

//#define stdout 0

typedef long int __fd_mask;

//struct timezone {
//        int     tz_minuteswest; /* minutes west of Greenwich */
//        int     tz_dsttime;     /* type of dst correction */
//};

/* Some versions of <linux/posix_types.h> define these macros.  */
#undef	__NFDBITS
#undef	__FDELT
#undef	__FDMASK
/* It's easier to assume 8-bit bytes than to get CHAR_BIT.  */
#define __NFDBITS	(8 * sizeof (__fd_mask))
#define	__FDELT(d)	((d) / __NFDBITS)
#define	__FDMASK(d)	((__fd_mask) 1 << ((d) % __NFDBITS))

#define BADCH   (int)'?'
#define BADARG  (int)':'
#define EMSG    ""

typedef unsigned char u_char;


struct utsname {
    char* sysname;    /* Operating system name (e.g., "Linux") */
    char* nodename;   /* Name within "some implementation-defined
                          network" */
    char* release;    /* OS release (e.g., "2.6.28") */
    char* version;    /* OS version */
    char* machine;    /* Hardware identifier */
#ifdef _GNU_SOURCE
    char* domainname; /* NIS or YP domain name */
#endif
};

//#define stderr 2

#define SOL_SOCKET	0xffff

/*
 * Option flags per-socket.
 */
#define	SO_DEBUG	0x0001		/* turn on debugging info recording */
#define	SO_ACCEPTCONN	0x0002		/* socket has had listen() */
#define	SO_REUSEADDR	0x0004		/* allow local address reuse */
#define	SO_KEEPALIVE	0x0008		/* keep connections alive */
#define	SO_DONTROUTE	0x0010		/* just use interface addresses */
#define	SO_BROADCAST	0x0020		/* permit sending of broadcast msgs */
#define	SO_USELOOPBACK	0x0040		/* bypass hardware when possible */
#define	SO_LINGER	0x0080		/* linger on close if data present */
#define	SO_OOBINLINE	0x0100		/* leave received OOB data in line */

/*
 * Additional options, not kept in so_options.
 */
#define SO_SNDBUF	0x1001		/* send buffer size */
#define SO_RCVBUF	0x1002		/* receive buffer size */
#define SO_SNDLOWAT	0x1003		/* send low-water mark */
#define SO_RCVLOWAT	0x1004		/* receive low-water mark */
#define SO_SNDTIMEO	0x1005		/* send timeout */
#define SO_RCVTIMEO	0x1006		/* receive timeout */
#define	SO_ERROR	0x1007		/* get error status and clear */
#define	SO_TYPE		0x1008		/* get socket type */

#define NSIG             32

#define SIGHUP           1
#define SIGINT           2
#define SIGQUIT          3
#define SIGILL           4
#define SIGTRAP          5
#define SIGABRT          6
#define SIGIOT           6
//#define SIGBUS           7
#define SIGFPE           8
#define SIGKILL          9
//#define SIGUSR1         10
#define SIGSEGV         11
//#define SIGUSR2         12
#define SIGPIPE         13
#define SIGALRM         14
#define SIGTERM         15
#define SIGSTKFLT       16
//#define SIGCHLD         17
//#define SIGCONT         18
//#define SIGSTOP         19
//#define SIGTSTP         20
#define SIGTTIN         21
#define SIGTTOU         22
//#define SIGURG          23
#define SIGXCPU         24
#define SIGXFSZ         25
#define SIGVTALRM       26
#define SIGPROF         27
#define SIGWINCH        28
//#define SIGIO           29
#define SIGPOLL         SIGIO
/*
#define SIGLOST         29
*/
#define SIGPWR          30
//#define SIGSYS          31
#define SIGUNUSED       31

//Included to support ffmpeg on Quest - smarotta, gfry
#define SA_RESTART 0x10000000

#define SI_MAX_SIZE 128
#define SI_PAD_SIZE ((SI_MAX_SIZE/sizeof(int)) - 3)

typedef int pid_t, sigval_t;

struct protoent {
    char  *p_name;       /* official protocol name */
    char **p_aliases;    /* alias list */
    int    p_proto;      /* protocol number */
};

typedef struct siginfo {
     int si_signo;
     int si_errno;
     int si_code;

     union {
          int _pad[SI_PAD_SIZE];

          /* kill() */
          struct {
               pid_t _pid; /* sender's pid */
               uid_t _uid; /* sender's uid */
          } _kill;

          /* POSIX.1b timers */
          struct {
               unsigned int _timer1;
               unsigned int _timer2;
          } _timer;

          /* POSIX.1b signals */
          struct {
               pid_t _pid; /* sender's pid */
               uid_t _uid; /* sender's uid */
               sigval_t _sigval;
          } _rt;

          /* SIGCHLD */
          struct {
               pid_t _pid; /* which child */
               uid_t _uid; /* sender's uid */
               int _status; /* exit code */
               clock_t _utime;
               clock_t _stime;
          } _sigchld;

          /* SIGILL, SIGFPE, SIGSEGV, SIGBUS */
          struct {
               void *_addr; /* faulting insn/memory ref. */
          } _sigfault;

          /* SIGPOLL */
          struct {
               int _band; /* POLL_IN, POLL_OUT, POLL_MSG */
               int _fd;
          } _sigpoll;
     } _sifields;
} siginfo_t;


  //struct sigaction {
//    void     (*sa_handler)(int);
//    void     (*sa_sigaction)(int, siginfo_t *, void *);
//    sigset_t   sa_mask;
//    int        sa_flags;
//    void     (*sa_restorer)(void);
//};


#define _SS_PAD1SIZE (_SS_ALIGNSIZE - sizeof(sa_family_t))
#define _SS_PAD2SIZE (_SS_MAXSIZE - (sizeof(sa_family_t)+ \
                      _SS_PAD1SIZE + _SS_ALIGNSIZE))

#if !defined(HAVE_STRUCT_SOCKADDR_STORAGE) && !defined(sockaddr_storage)
#define sockaddr_storage sockaddr_in
#endif

#define    IFNAMSIZ    16

struct ifmap {
    unsigned long   mem_start;
    unsigned long   mem_end;
    unsigned short  base_addr;
    unsigned char   irq;
    unsigned char   dma;
    unsigned char   port;
};

struct ifreq {
    char ifr_name[IFNAMSIZ]; /* Interface name */
    union {
      struct sockaddr ifr_addr;
      struct sockaddr ifr_dstaddr;
      struct sockaddr ifr_broadaddr;
      struct sockaddr ifr_netmask;
      struct sockaddr ifr_hwaddr;
      short           ifr_flags;
      int             ifr_ifindex;
      int             ifr_metric;
      int             ifr_mtu;
      struct ifmap    ifr_map;
      char            ifr_slave[IFNAMSIZ];
      char            ifr_newname[IFNAMSIZ];
      char           *ifr_data;
    };
};

#define ETHTOOL_GDRVINFO        0x00000003 /* Get driver info. */

#define ETHTOOL_BUSINFO_LEN     32

struct ethtool_drvinfo {
  uint32_t     cmd;
  char    driver[32];     /* driver short name, "tulip", "eepro100" */
  char    version[32];    /* driver version string */
  char    fw_version[32]; /* firmware version string, if applicable */
  char    bus_info[ETHTOOL_BUSINFO_LEN];  /* Bus info for this IF. */
  /* For PCI devices, use pci_dev->slot_name. */
  char    reserved1[32];
  char    reserved2[16];
  uint32_t     n_stats;        /* number of u64's from ETHTOOL_GSTATS */
  uint32_t     testinfo_len;
  uint32_t     eedump_len;     /* Size of data from ETHTOOL_GEEPROM (bytes) */
  uint32_t     regdump_len;    /* Size of data from ETHTOOL_GREGS (bytes) */
};


struct FTW {
    int base;
    int level;
};


# define FTW_PHYS	1
# define FTW_MOUNT	2
# define FTW_CHDIR	4
# define FTW_DEPTH	8



//struct stat {
//    dev_t     st_dev;     /* ID of device containing file */
//    ino_t     st_ino;     /* inode number */
//    mode_t    st_mode;    /* protection */
//    nlink_t   st_nlink;   /* number of hard links */
//    uid_t     st_uid;     /* user ID of owner */
//    gid_t     st_gid;     /* group ID of owner */
//    dev_t     st_rdev;    /* device ID (if special file) */
//    off_t     st_size;    /* total size, in bytes */
//    blksize_t st_blksize; /* blocksize for file system I/O */
//    blkcnt_t  st_blocks;  /* number of 512B blocks allocated */
//    time_t    st_atime;   /* time of last access */
//    time_t    st_mtime;   /* time of last modification */
//    time_t    st_ctime;   /* time of last status change */
//};

struct sockaddr_nl {
    sa_family_t     nl_family;  /* AF_NETLINK */
    unsigned short  nl_pad;     /* Zero. */
    pid_t           nl_pid;     /* Process ID. */
    __u32           nl_groups;  /* Multicast groups mask. */
};

#define IF_NAMESIZE 16

struct nlmsghdr {
    __u32 nlmsg_len;    /* Length of message including header. */
    __u16 nlmsg_type;   /* Type of message content. */
    __u16 nlmsg_flags;  /* Additional flags. */
    __u32 nlmsg_seq;    /* Sequence number. */
    __u32 nlmsg_pid;    /* PID of the sending process. */
};

struct iovec {
    void  *iov_base;    /* Starting address */
    size_t iov_len;     /* Number of bytes to transfer */
};

struct rtmsg {
    unsigned char rtm_family;   /* Address family of route */
    unsigned char rtm_dst_len;  /* Length of destination */
    unsigned char rtm_src_len;  /* Length of source */
    unsigned char rtm_tos;      /* TOS filter */

   unsigned char rtm_table;    /* Routing table ID */
    unsigned char rtm_protocol; /* Routing protocol; see below */
    unsigned char rtm_scope;    /* See below */
    unsigned char rtm_type;     /* See below */

   unsigned int  rtm_flags;
};

struct ifaddrmsg {
    unsigned char ifa_family;    /* Address type */
    unsigned char ifa_prefixlen; /* Prefixlength of address */
    unsigned char ifa_flags;     /* Address flags */
    unsigned char ifa_scope;     /* Address scope */
    int           ifa_index;     /* Interface index */
};

struct ifinfomsg {
    unsigned char  ifi_family; /* AF_UNSPEC */
    unsigned short ifi_type;   /* Device type */
    int            ifi_index;  /* Interface index */
    unsigned int   ifi_flags;  /* Device flags  */
    unsigned int   ifi_change; /* change mask */
};

struct rtattr {
    unsigned short rta_len;    /* Length of option */
    unsigned short rta_type;   /* Type of option */
    /* Data follows */
};

/*
 * Index offsets for sockaddr array for alternate internal encoding.
 */
#define RTAX_DST        0       /* destination sockaddr present */
#define RTAX_GATEWAY    1       /* gateway sockaddr present */
#define RTAX_NETMASK    2       /* netmask sockaddr present */
#define RTAX_GENMASK    3       /* cloning mask sockaddr present */

#define NLM_F_REQUEST 1

enum {
        RTM_BASE   = 16,
#define RTM_BASE   RTM_BASE

        RTM_NEWLINK        = 16,
#define RTM_NEWLINK        RTM_NEWLINK
        RTM_DELLINK,
#define RTM_DELLINK        RTM_DELLINK
        RTM_GETLINK,
#define RTM_GETLINK        RTM_GETLINK
        RTM_SETLINK,
#define RTM_SETLINK        RTM_SETLINK

        RTM_NEWADDR        = 20,
#define RTM_NEWADDR        RTM_NEWADDR
        RTM_DELADDR,
#define RTM_DELADDR        RTM_DELADDR
        RTM_GETADDR,
#define RTM_GETADDR        RTM_GETADDR

        RTM_NEWROUTE       = 24,
#define RTM_NEWROUTE       RTM_NEWROUTE
        RTM_DELROUTE,
#define RTM_DELROUTE       RTM_DELROUTE
        RTM_GETROUTE,
#define RTM_GETROUTE       RTM_GETROUTE

        RTM_NEWNEIGH       = 28,
#define RTM_NEWNEIGH       RTM_NEWNEIGH
        RTM_DELNEIGH,
#define RTM_DELNEIGH       RTM_DELNEIGH
        RTM_GETNEIGH,
#define RTM_GETNEIGH       RTM_GETNEIGH

        RTM_NEWRULE        = 32,
#define RTM_NEWRULE        RTM_NEWRULE
        RTM_DELRULE,
#define RTM_DELRULE        RTM_DELRULE
        RTM_GETRULE,
#define RTM_GETRULE        RTM_GETRULE

        RTM_NEWQDISC       = 36,
#define RTM_NEWQDISC       RTM_NEWQDISC
        RTM_DELQDISC,
#define RTM_DELQDISC       RTM_DELQDISC
        RTM_GETQDISC,
#define RTM_GETQDISC       RTM_GETQDISC

        RTM_NEWTCLASS      = 40,
#define RTM_NEWTCLASS      RTM_NEWTCLASS
        RTM_DELTCLASS,
#define RTM_DELTCLASS      RTM_DELTCLASS
        RTM_GETTCLASS,
#define RTM_GETTCLASS      RTM_GETTCLASS

        RTM_NEWTFILTER     = 44,
#define RTM_NEWTFILTER     RTM_NEWTFILTER
        RTM_DELTFILTER,
#define RTM_DELTFILTER     RTM_DELTFILTER
        RTM_GETTFILTER,
#define RTM_GETTFILTER     RTM_GETTFILTER

        RTM_NEWACTION      = 48,
#define RTM_NEWACTION   RTM_NEWACTION
        RTM_DELACTION,
#define RTM_DELACTION   RTM_DELACTION
        RTM_GETACTION,
#define RTM_GETACTION   RTM_GETACTION

        RTM_NEWPREFIX      = 52,
#define RTM_NEWPREFIX      RTM_NEWPREFIX

        RTM_GETMULTICAST = 58,
#define RTM_GETMULTICAST RTM_GETMULTICAST

        RTM_GETANYCAST     = 62,
#define RTM_GETANYCAST     RTM_GETANYCAST

        RTM_NEWNEIGHTBL    = 64,
#define RTM_NEWNEIGHTBL    RTM_NEWNEIGHTBL
        RTM_GETNEIGHTBL    = 66,
#define RTM_GETNEIGHTBL    RTM_GETNEIGHTBL
        RTM_SETNEIGHTBL,
#define RTM_SETNEIGHTBL    RTM_SETNEIGHTBL

        RTM_NEWNDUSEROPT = 68,
#define RTM_NEWNDUSEROPT RTM_NEWNDUSEROPT

        RTM_NEWADDRLABEL = 72,
#define RTM_NEWADDRLABEL RTM_NEWADDRLABEL
        RTM_DELADDRLABEL,
#define RTM_DELADDRLABEL RTM_DELADDRLABEL
        RTM_GETADDRLABEL,
#define RTM_GETADDRLABEL RTM_GETADDRLABEL

        RTM_GETDCB = 78,
#define RTM_GETDCB RTM_GETDCB
        RTM_SETDCB,
#define RTM_SETDCB RTM_SETDCB

        __RTM_MAX,
#define RTM_MAX            (((__RTM_MAX + 3) & ~3) - 1)
};


#endif
