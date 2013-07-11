/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "arch/i386.h"
#include "kernel.h"
#include "mem/mem.h"
#include "fs/filesys.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "sched/sched.h"
#include "util/printf.h"
#include "util/screen.h"
#include "util/debug.h"
#include "util/circular.h"
#include "sched/sched.h"
#include "sched/proc.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "linux_socket.h"
#include "select.h"
#include "arch/i386-div64.h"
#include "interrupt_handler.h"
#include "fcntl.h"

#ifdef USE_VMX
#include "vm/shm.h"
#endif

//#define DEBUG_SOCKET

#ifdef DEBUG_SOCKET
#define DLOG(fmt,...) DLOG_PREFIX("socket_syscall",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

typedef void (*sys_call_ptr_t) (void);

static void
udp_recv_callback (void *arg, struct udp_pcb *upcb, struct pbuf *p,
               struct ip_addr *addr, uint16 port)
{
  DLOG ("UDP packet received from: %s:%d",
        inet_ntoa (* ((struct in_addr *) addr)), port);
  DLOG ("Total length: %d", p->tot_len);

  fd_table_entry_t * fd_ent = (fd_table_entry_t *) arg;
  udp_recv_buf_t b;

  if (p == NULL) {
    DLOG ("udp_recv pbuf is NULL");
    return;
  }

  b.buf = p;
  b.addr.sin_family = AF_INET;
  b.addr.sin_port = htons (port);
  b.addr.sin_addr.s_addr = addr->addr;
  b.bytes_read = 0;
  if (circular_insert_nowait (fd_ent->udp_recv_buf_circ, &b) == -1) {
    DLOG ("udp_recv_buf is full, packet dropped");
    pbuf_free (p);
  }

  return;
}

static err_t
tcp_recv_callback (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  udp_recv_buf_t b;
  fd_table_entry_t * fd_ent = (fd_table_entry_t *) arg;

  if (err != ERR_OK) {
    DLOG ("TCP Callback error: %d", err);
  } else {
    if (p == NULL) {
      /* Remote side closes connection */
      /* --??-- What should be done in this case? */
      DLOG ("Connection closed by remote host");
      return err;
    }

    b.buf = p;
    b.addr.sin_family = AF_INET;
    b.addr.sin_port = htons (tpcb->remote_port);
    b.addr.sin_addr.s_addr = tpcb->remote_ip.addr;
    b.bytes_read = 0;
    if (circular_insert_nowait (fd_ent->tcp_recv_buf_circ, &b) == -1) {
      DLOG ("tcp_recv_buf is full, packet dropped");
      tcp_recved (tpcb, p->tot_len);
      pbuf_free (p);
    }
  }

  return err;
}

static int sys_call_tcp_sent_status = 0;

static err_t
tcp_sent_callback (void *arg, struct tcp_pcb *tpcb, uint16 len)
{
  sys_call_tcp_sent_status = len;
  return ERR_OK;
}

static int
sys_call_open_socket (int domain, int type, int protocol)
{
  int sockfd = 0;
  quest_tss * tss;
  task_id cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  if (domain != AF_INET) {
    logger_printf ("sys_call_open_socket: Domain not supported\n");
    return -1;
  }

  switch (type) {
    case SOCK_DGRAM :
      if ((sockfd = find_fd (tss)) == -1) {
        DLOG ("No free file descriptor available");
      } else {
        struct udp_pcb * upcb = udp_new ();
        if (upcb == NULL) {
          DLOG ("Cannot allocate UDP PCB");
          return -1;
        }
        tss->fd_table[sockfd].task = tss;
        tss->fd_table[sockfd].type = FD_TYPE_UDP;
        tss->fd_table[sockfd].entry = (void *) upcb;
        tss->fd_table[sockfd].udp_recv_buf_circ = kzalloc (sizeof (circular));
        tss->fd_table[sockfd].udp_recv_buf = kzalloc (sizeof (udp_recv_buf_t) *
                                                      UDP_RECV_BUF_LEN);
        udp_recv ((struct udp_pcb *) tss->fd_table[sockfd].entry, udp_recv_callback,
                  (void *) (&tss->fd_table[sockfd]));
        if ((tss->fd_table[sockfd].udp_recv_buf_circ == 0) ||
            (tss->fd_table[sockfd].udp_recv_buf == 0)) {
          DLOG ("kzalloc failed!");
          return -1;
        }
        circular_init (tss->fd_table[sockfd].udp_recv_buf_circ,
                       (void *) tss->fd_table[sockfd].udp_recv_buf,
                       UDP_RECV_BUF_LEN,
                       sizeof (udp_recv_buf_t));
        
        DLOG ("New UDP socket descriptor: %d", sockfd);
      }
      break;
    case SOCK_STREAM :
      if ((sockfd = find_fd (tss)) == -1) {
        DLOG ("No free file descriptor available");
      } else {
        struct tcp_pcb * tpcb = tcp_new ();
        if (tpcb == NULL) {
          DLOG ("Cannot allocate TCP PCB");
          return -1;
        }
        tss->fd_table[sockfd].task = tss;
        tss->fd_table[sockfd].type = FD_TYPE_TCP;
        tss->fd_table[sockfd].entry = (void *) tpcb;
        tss->fd_table[sockfd].tcp_recv_buf_circ = kzalloc (sizeof (circular));
        tss->fd_table[sockfd].tcp_accept_circ = kzalloc (sizeof (circular));
        tss->fd_table[sockfd].tcp_recv_buf = kzalloc (sizeof (tcp_recv_buf_t) *
                                                      TCP_RECV_BUF_LEN);
        tss->fd_table[sockfd].tcp_accept_buf = kzalloc (sizeof (int) *
                                                        TCP_ACCEPT_LEN);
        if ((tss->fd_table[sockfd].tcp_recv_buf_circ == 0) ||
            (tss->fd_table[sockfd].tcp_recv_buf == 0) ||
            (tss->fd_table[sockfd].tcp_accept_circ == 0) ||
            (tss->fd_table[sockfd].tcp_accept_buf == 0)) {
          DLOG ("kzalloc failed!");
          return -1;
        }
        circular_init (tss->fd_table[sockfd].tcp_recv_buf_circ,
                       (void *) tss->fd_table[sockfd].tcp_recv_buf,
                       TCP_RECV_BUF_LEN,
                       sizeof (tcp_recv_buf_t));
        
        circular_init (tss->fd_table[sockfd].tcp_accept_circ,
                       (void *) tss->fd_table[sockfd].tcp_accept_buf,
                       TCP_ACCEPT_LEN,
                       sizeof (int));

        tcp_arg (tpcb, (void *) (&tss->fd_table[sockfd]));
        DLOG ("New TCP socket descriptor: %d", sockfd);
      }
      break;
    default :
      logger_printf ("Socket type %d is not supported\n", type);
      return -1;
  }

  return sockfd;
}

static int
sys_call_close (int filedes)
{
  quest_tss * tss;
  err_t err;
  task_id cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  if(filedes < 3) {
    return 0;
  }
  
  switch (tss->fd_table[filedes].type) {
  case FD_TYPE_UDP :
    DLOG ("close UDP socket %d", filedes);
    udp_remove ((struct udp_pcb *) tss->fd_table[filedes].entry);
    tss->fd_table[filedes].entry = NULL;
    kfree (tss->fd_table[filedes].udp_recv_buf);
    kfree (tss->fd_table[filedes].udp_recv_buf_circ);
    break;
  case FD_TYPE_TCP :
    DLOG ("close TCP socket %d", filedes);
    if ((err = tcp_close ((struct tcp_pcb *) tss->fd_table[filedes].entry)) != ERR_OK) {
      DLOG ("TCP PCB close failed: %d", err);
      return -1;
    }
    tss->fd_table[filedes].entry = NULL;
    kfree (tss->fd_table[filedes].tcp_recv_buf);
    kfree (tss->fd_table[filedes].tcp_recv_buf_circ);
    kfree (tss->fd_table[filedes].tcp_accept_buf);
    kfree (tss->fd_table[filedes].tcp_accept_circ);
    break;
  case FD_TYPE_FILE:
    free_fd_table_file_entry(tss->fd_table[filedes].entry);
    tss->fd_table[filedes].entry = NULL;
    break;
  default :
    logger_printf ("Socket or file type %d not supported in close\n",
                   tss->fd_table[filedes].type);
    return -1;
  }
 
  return 0;
}

static int
sys_call_bind (int sockfd, uint32_t addr, uint16_t port)
{
  quest_tss * tss;
  err_t err;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_UDP :
      DLOG ("UDP socket %d bind to: %s:%d", sockfd,
            inet_ntoa (* ((struct in_addr *) &addr)), ntohs (port));
      if ((err = udp_bind ((struct udp_pcb *) fd_ent.entry,
                           (struct ip_addr *) &addr, ntohs (port))) != ERR_OK) {
        DLOG ("UDP bind failed: %d", err);
        return -1;
      }
      break;
    case FD_TYPE_TCP :
      DLOG ("TCP socket %d bind to: %s:%d", sockfd,
            inet_ntoa (* ((struct in_addr *) &addr)), ntohs (port));
      if ((err = tcp_bind ((struct tcp_pcb *) fd_ent.entry,
                           (struct ip_addr *) &addr, ntohs (port))) != ERR_OK) {
        DLOG ("TCP bind failed: %d", err);
        return -1;
      }
      break;
    default :
      logger_printf ("Socket type %d not supported in bind\n",
                     fd_ent.type);
      return -1;
  }

  return 0;
}

static int sys_call_tcp_connect_status = 0;

static err_t
tcp_connected (void * arg, struct tcp_pcb * tpcb, err_t err)
{
  if (err != ERR_OK) {
    DLOG ("TCP connect failed: %d", err);
    sys_call_tcp_connect_status = -1;
  } else {
    DLOG ("TCP connection established");
    /* Register receive callback once connection is established */
    tcp_recv (tpcb, tcp_recv_callback);
    sys_call_tcp_connect_status = 1;
  }

  return err;
}

static int
sys_call_connect (int sockfd, uint32_t addr, uint16_t port)
{
  quest_tss * tss;
  err_t err;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_UDP :
      if ((err = udp_connect ((struct udp_pcb *) fd_ent.entry,
                              (struct ip_addr *) &addr, ntohs (port))) != ERR_OK) {
        DLOG ("UDP connect failed: %d", err);
        return -1;
      }
      break;
    case FD_TYPE_TCP :
      sys_call_tcp_connect_status = 0;
      if ((err = tcp_connect ((struct tcp_pcb *) fd_ent.entry,
                              (struct ip_addr *) &addr, ntohs (port), tcp_connected)) != ERR_OK) {
        DLOG ("TCP connect failed: %d", err);
        return -1;
      }
      DLOG ("TCP Connecting to: %s:%d ...",
            inet_ntoa (* ((struct in_addr *) &addr)), ntohs (port));
      sti ();
      while (!sys_call_tcp_connect_status);
      cli ();
      if (sys_call_tcp_connect_status == -1)
        return -1;
      break;
    default :
      logger_printf ("Socket type %d not supported in connect\n",
                     fd_ent.type);
      return -1;
  }

  return 0;
}

static err_t
tcp_accept_callback (void *arg, struct tcp_pcb *new_tpcb, err_t err)
{
  fd_table_entry_t * fd_ent = (fd_table_entry_t *) arg;
  quest_tss * tss = fd_ent->task;
  int new_sockfd = 0;

  if (err != ERR_OK) {
    logger_printf ("Accept error returned by Lwip");
    return err;
  } else {
    if ((new_sockfd = find_fd (tss)) == -1 ) {
      DLOG ("Cannot allocate file descriptor");
      return -1;
    }

    tss->fd_table[new_sockfd].task = tss;
    tss->fd_table[new_sockfd].entry = (void *) new_tpcb;
    tss->fd_table[new_sockfd].type = FD_TYPE_TCP;
    tss->fd_table[new_sockfd].tcp_recv_buf_circ = kzalloc (sizeof (circular));
    tss->fd_table[new_sockfd].tcp_accept_circ = kzalloc (sizeof (circular));
    tss->fd_table[new_sockfd].tcp_recv_buf = kzalloc (sizeof (tcp_recv_buf_t) *
                                                      TCP_RECV_BUF_LEN);
    tss->fd_table[new_sockfd].tcp_accept_buf = kzalloc (sizeof (int) *
                                                        TCP_ACCEPT_LEN);
    if ((tss->fd_table[new_sockfd].tcp_recv_buf_circ == 0) ||
        (tss->fd_table[new_sockfd].tcp_recv_buf == 0) ||
        (tss->fd_table[new_sockfd].tcp_accept_circ == 0) ||
        (tss->fd_table[new_sockfd].tcp_accept_buf == 0)) {
      DLOG ("kzalloc failed!");
      return -1;
    }
    circular_init (tss->fd_table[new_sockfd].tcp_recv_buf_circ,
                   (void *) tss->fd_table[new_sockfd].tcp_recv_buf,
                   TCP_RECV_BUF_LEN,
                   sizeof (tcp_recv_buf_t));

    circular_init (tss->fd_table[new_sockfd].tcp_accept_circ,
                   (void *) tss->fd_table[new_sockfd].tcp_accept_buf,
                   TCP_ACCEPT_LEN,
                   sizeof (int));

    tcp_arg (new_tpcb, (void *) (&tss->fd_table[new_sockfd]));
    /* Register receive callback once connection is accepted */
    tcp_recv (new_tpcb, tcp_recv_callback);

    if (circular_insert_nowait (fd_ent->tcp_accept_circ, &new_sockfd) == -1) {
      DLOG ("TCP connection accept buffer is full");
      /* Which error code shall we return here? */
      return -1;
    }

    DLOG ("New socket %d inserted to accept queue", new_sockfd);
    /* Register receive callback once connection is accepted */
    //tcp_recv (new_pcb, tcp_recv_callback);
  }

  return err;
}

static int
sys_call_listen (int sockfd, int backlog)
{
  quest_tss * tss;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);
  struct tcp_pcb *new_pcb;

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_TCP :
      if (fd_ent.entry == NULL) {
        DLOG ("Listen Failed, NULL TCP PCB");
        return -1;
      }
      DLOG ("Listening on socket %d", sockfd);
      new_pcb = tcp_listen ((struct tcp_pcb *) fd_ent.entry);
      fd_ent.entry = (void *) new_pcb;
      tcp_accept (new_pcb, tcp_accept_callback);
      break;
    default :
      logger_printf ("Socket type %d not supported in listen\n",
                     fd_ent.type);
      return -1;
  }

  return 0;
}

static int
sys_call_accept (int sockfd, void *addr, void *len)
{
  quest_tss * tss;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);
  struct tcp_pcb *new_tpcb;
  int new_sockfd = -1;

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_TCP :
      DLOG ("Socket %d waiting for connection...", sockfd);

      lock_kernel ();
      circular_remove (fd_ent.tcp_accept_circ, &new_sockfd);
      unlock_kernel ();

      if (new_sockfd == -1) {
        DLOG ("Could not find accepted socket!");
        return -1;
      }

      new_tpcb = tss->fd_table[new_sockfd].entry;

      if (addr) {
        ((struct sockaddr_in *) addr)->sin_family = AF_INET;
        ((struct sockaddr_in *) addr)->sin_port = htons (new_tpcb->remote_port);
        ((struct sockaddr_in *) addr)->sin_addr.s_addr = new_tpcb->remote_ip.addr;
      }
      DLOG ("New connection socked %d accepted: %s:%d", new_sockfd,
            inet_ntoa (* ((struct in_addr *) &(new_tpcb->remote_ip))),
            new_tpcb->remote_port);

      if (len)
        *((socklen_t *) len) = sizeof (struct sockaddr_in);
      tcp_accepted ((struct tcp_pcb *) fd_ent.entry);
      break;
    default :
      logger_printf ("Socket type %d not supported in accept\n",
                     fd_ent.type);
      return -1;
  }

  return new_sockfd;
}

static int
sys_call_write (int filedes, const void *buf, int nbytes)
{
  quest_tss * tss;
  err_t err;
  struct pbuf *p = NULL;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);
  uint16 sndbuf_len = 0;
  int nbytes_sent = nbytes;
  static bool first = TRUE;
  extern int print_buffer(char *pch, int length);

#ifdef USE_VMX
  uint32 cpu;
  cpu = get_pcpu_id ();
  if (shm_screen_initialized) {
    if ((shm->virtual_display.cursor[cpu].x == -1) &&
        (shm->virtual_display.cursor[cpu].y == -1)) {
      shm->virtual_display.cursor[cpu].x = 0;
      shm->virtual_display.cursor[cpu].y = 0;
      
    }
  } 
  if (first) {
    splash_screen ();
    first = FALSE;
  }
#else
  if (first) { splash_screen (); first = FALSE; }
#endif

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  /* HACK for STDOUT and STDERR */
  if ((filedes == 1) || (filedes == 2)) {
    print_buffer((char*)buf, nbytes);
    return nbytes;
  }

  fd_ent = tss->fd_table[filedes];

  switch (fd_ent.type) {
    case FD_TYPE_UDP :
      p = pbuf_alloc (PBUF_TRANSPORT, nbytes, PBUF_RAM);
      if (p == NULL) {
        DLOG ("pbuf allocation failed");
        return -1;
      }
      /* --!!-- Remove memcpy if performance or memory becomes a problem */
      memcpy (p->payload, buf, nbytes);
      DLOG ("Sending %d bytes on UDP socket %d", nbytes, filedes);
      if ((err = udp_send ((struct udp_pcb *) fd_ent.entry, p)) != ERR_OK) {
        DLOG ("UDP sent failed : %d", err);
        pbuf_free (p);
        return -1;
      }
      pbuf_free (p);
      break;
    case FD_TYPE_TCP :
      DLOG ("Sending %d bytes on TCP socket %d", nbytes, filedes);
      nbytes_sent = 0;
      tcp_sent ((struct tcp_pcb *) fd_ent.entry, tcp_sent_callback);
TCP_SENDING:
      sys_call_tcp_sent_status = 0;
      if ((sndbuf_len = tcp_sndbuf ((struct tcp_pcb *) fd_ent.entry)) < (nbytes - nbytes_sent)) {
        /* --!!-- We should wait and re-check here */
        DLOG ("Not enough space in output queue");
      } else {
        sndbuf_len = nbytes - nbytes_sent;
      }

      int count = 0;
      while ((err = tcp_write ((struct tcp_pcb *) fd_ent.entry,
                               ((const char *)buf) + nbytes_sent, sndbuf_len, 1))
             != ERR_OK ) {
        DLOG ("tcp_write failed: %d", err);
        DLOG ("nbytes_sent=%d", nbytes_sent);
        if (err == ERR_MEM) {
          /* Retry if ERR_MEM returned */
          tsc_delay_usec (5000);
          if (++count >= 5) {
            return -1;
          }
          continue;
        }
        return -1;
      }
      tcp_output ((struct tcp_pcb *) fd_ent.entry);
      sti ();
      while (!sys_call_tcp_sent_status);
      cli ();
      DLOG ("TCP %d bytes sent", nbytes_sent);
      nbytes_sent += sys_call_tcp_sent_status;
      if (nbytes_sent != nbytes) goto TCP_SENDING;
      break;
    default :
      logger_printf ("Socket type %d not supported in write\n",
                     fd_ent.type);
      return -1;
  }

  return nbytes_sent;
}

static int
sys_call_sendto (int sockfd, const void *buf, int nbytes, uint32 addr, uint16 port)
{
  quest_tss * tss;
  err_t err;
  struct pbuf *p = NULL;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_UDP :
      p = pbuf_alloc (PBUF_TRANSPORT, nbytes, PBUF_RAM);
      if (p == NULL) {
        DLOG ("pbuf allocation failed");
        return -1;
      }
      /* --!!-- Remove memcpy if performance or memory becomes a problem */
      memcpy (p->payload, buf, nbytes);
      DLOG ("pbuf.tot_len=%d, pbuf.len=%d", p->tot_len, p->len);
      /* lwip says: port should be in the same byte order as in PCB... */
      /* So, it seems that it's in host byte order */
      if ((err = udp_sendto ((struct udp_pcb *) fd_ent.entry, p,
                             (struct ip_addr *) &addr, ntohs (port))) != ERR_OK) {
        logger_printf ("UDP sendto failed : %d\n", err);
        pbuf_free (p);
        return -1;
      }
      DLOG ("UDP packet sent to: %s:%d",
            inet_ntoa (* ((struct in_addr *) &addr)), ntohs (port));
      pbuf_free (p);
      break;
    case FD_TYPE_TCP :
      /* call write directly for connection-based socket */
      return sys_call_write (sockfd, buf, nbytes);
    default :
      logger_printf ("Socket type %d not supported in sendto\n",
                     fd_ent.type);
      return -1;
  }

  return nbytes;
}

static udp_recv_buf_t udpb = {.buf = NULL, .bytes_read = 0};
static tcp_recv_buf_t tcpb = {.buf = NULL, .bytes_read = 0};

static int
sys_call_recv (int sockfd, void *buf, int nbytes, void *addr, void *len)
{
  quest_tss * tss;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);
  int nbytes_recvd = 0;
  struct pbuf *q = NULL;
  char *b = buf;
  int buf_index = 0;

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  fd_ent = tss->fd_table[sockfd];

  if(!fd_ent.entry) {
    return -1;
  }

  switch (fd_ent.type) {
    case FD_TYPE_UDP :
      DLOG ("sys_call_recv: Receiving UDP data from socket %d", sockfd);
      if (udpb.buf == NULL) {
        lock_kernel ();
        if(fd_ent.flags & O_NONBLOCK) {
          circular_remove_nowait (fd_ent.udp_recv_buf_circ, &udpb);
          if(udpb.buf == NULL) {
            unlock_kernel();
            return 0;
          }
        }
        else {
          circular_remove (fd_ent.udp_recv_buf_circ, &udpb);
        }
        
        unlock_kernel ();
      }

      q = udpb.buf;
      DLOG ("Data available, Total length: %d, length: %d, User buffer length: %d",
            q->tot_len, q->len, nbytes);
      DLOG ("Buffer read: %d", udpb.bytes_read);
      /* Calculate index */
      buf_index = udpb.bytes_read;
      while (buf_index >= q->len) {
        if (q->next) {
          buf_index -= q->len;
          q = q->next;
        } else {
          DLOG ("Received length greater than pbuf total");
          pbuf_free (udpb.buf);
          udpb.buf = NULL;
          udpb.bytes_read = 0;
          return -1;
        }
      }
      /* Read from pbuf */
      while (q) {
        if ((nbytes_recvd + q->len - buf_index) > nbytes) {
          DLOG ("UDP user buffer smaller than pbuf");
          memcpy (b, ((uint8 *) q->payload) + buf_index, nbytes - nbytes_recvd);
          udpb.bytes_read += (nbytes - nbytes_recvd);
          return nbytes;
        }
        memcpy (b, ((uint8 *) q->payload) + buf_index, q->len - buf_index);
        b += (q->len - buf_index);
        nbytes_recvd += (q->len - buf_index);
        q = q->next;
        buf_index = 0;
      }
      DLOG ("Bytes received: %d", nbytes_recvd);
      if (addr)
        memcpy (addr, &(udpb.addr), sizeof (struct sockaddr_in));
      if (len)
        *((socklen_t *) len) = sizeof (struct sockaddr_in);
      pbuf_free (udpb.buf);
      udpb.buf = NULL;
      udpb.bytes_read = 0;
      break;
    case FD_TYPE_TCP :
      DLOG ("sys_call_recv: Receiving TCP data from socket %d", sockfd);
      if (tcpb.buf == NULL) {
        lock_kernel ();
        if(fd_ent.flags & O_NONBLOCK) {
          logger_printf("tcp remove no wait called\n");
          circular_remove_nowait (fd_ent.tcp_recv_buf_circ, &tcpb);
          if(tcpb.buf == NULL) {
            unlock_kernel();
            return 0;
          }
        }
        else {
          logger_printf("tcp remove called\n");
          circular_remove (fd_ent.tcp_recv_buf_circ, &tcpb);
        }
        
        unlock_kernel ();
      }

      q = tcpb.buf;
      DLOG ("Data available, Total length: %d, length: %d, User buffer length: %d",
            q->tot_len, q->len, nbytes);
      DLOG ("Buffer read: %d", tcpb.bytes_read);
      /* Calculate index */
      buf_index = tcpb.bytes_read;
      while (buf_index >= q->len) {
        if (q->next) {
          buf_index -= q->len;
          q = q->next;
        } else {
          DLOG ("Received length greater than pbuf total");
          pbuf_free (tcpb.buf);
          tcpb.buf = NULL;
          tcpb.bytes_read = 0;
          return -1;
        }
      }
      /* Read from pbuf */
      while (q) {
        if ((nbytes_recvd + q->len - buf_index) > nbytes) {
          DLOG ("TCP user buffer smaller than pbuf");
          memcpy (b, ((uint8 *) q->payload) + buf_index, nbytes - nbytes_recvd);
          tcpb.bytes_read += (nbytes - nbytes_recvd);
          return nbytes;
        }
        memcpy (b, ((uint8 *) q->payload) + buf_index, q->len - buf_index);
        b += (q->len - buf_index);
        nbytes_recvd += (q->len - buf_index);
        q = q->next;
        buf_index = 0;
      }
      DLOG ("Bytes received: %d", nbytes_recvd);
      if (addr)
        memcpy (addr, &(tcpb.addr), sizeof (struct sockaddr_in));
      if (len)
        *((socklen_t *) len) = sizeof (struct sockaddr_in);
      tcp_recved ((struct tcp_pcb *) fd_ent.entry, tcpb.buf->tot_len);
      pbuf_free (tcpb.buf);
      tcpb.buf = NULL;
      tcpb.bytes_read = 0;
      break;
    default :
      logger_printf ("Socket type %d not supported in recv\n",
                     fd_ent.type);
      return -1;
  }

  return nbytes_recvd;
}

static int
sys_call_select (int maxfdp1, fd_set * readfds, fd_set * writefds,
                 fd_set * exceptfds, struct timeval * tvptr)
{
  int i = 0, count = 0;
  quest_tss * tss;
  task_id cur = percpu_read (current_task);
  fd_set read_ready;
  bool ready = FALSE;
  int wait_count = 0;

  FD_ZERO (&read_ready);

  lock_kernel ();

  if (!cur) {
    logger_printf ("No current task\n");
    count = 0;
    goto finish;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    count = 0;
    goto finish;
  }

  if (tvptr != NULL)
    DLOG ("Time out: %d seconds, %d microseconds", tvptr->tv_sec, tvptr->tv_usec);

  /* If no sets specified, select becomes "sleep" */
  if ((readfds == NULL) && (writefds == NULL)) {
    if (tvptr != NULL) {
      sched_usleep (tvptr->tv_sec * 1000000LL + tvptr->tv_usec);
      count = 0;
    }
    goto finish;
  }

#ifdef DEBUG_SOCKET
  /* Debug info */
  DLOG("maxfdp1 = %d", maxfdp1);
  DLOG ("Select called on readfds:");
  logger_printf ("  ");
  for (i = 0; i < MAX_FD; i++) {
    if (FD_ISSET (i, readfds)) {
      logger_printf ("%d ", i);
    }
  }
  logger_printf ("\n");
#endif

#if 1

check_readfds:

  if (readfds) {
    for (i = 0; i < maxfdp1; i++) {
      if (FD_ISSET (i, readfds)) {
        switch (tss->fd_table[i].type) {
          case FD_TYPE_UDP :
            if (udpb.buf) {
              ready = TRUE;
              FD_SET(i, &read_ready);
              count++;
            } else if (circular_remove_nowait (tss->fd_table[i].udp_recv_buf_circ,
                                               &udpb) != -1) {
              ready = TRUE;
              FD_SET(i, &read_ready);
              count++;
            } else {
              continue;
            }
            break;
          case FD_TYPE_TCP :
            if (tcpb.buf) {
              ready = TRUE;
              FD_SET(i, &read_ready);
              count++;
            } else if (circular_remove_nowait (tss->fd_table[i].tcp_recv_buf_circ,
                                               &tcpb) != -1) {
              ready = TRUE;
              FD_SET(i, &read_ready);
              count++;
            } else {
              continue;
            }
            break;
          default:
            DLOG ("File descriptor type unsupported in select, fd = %d, type = %d\n",
                  i, tss->fd_table[i].type);
            return -1;
        }
      }
    }
  }

  if (tvptr == NULL) goto skip_waiting;

  /* I'm lazy, just sleep 16 times here... */
  if (!ready && (wait_count < 16)) {
    sched_usleep ((tvptr->tv_sec * 1000000LL + tvptr->tv_usec) >> 4);
    wait_count++;
    goto check_readfds;
  } else {
    /* Oops! Time out! */
    if (!ready) {
      count = 0;
      DLOG ("Select timeout");
    } else {
      /* Something is ready */
      DLOG ("readfds has fd ready");
    }
  }

skip_waiting:
  /* Now, some read fds are ready. We can return since we ignore write fds for now. */
  /* Replace old readfds with read_ready set */
  memcpy (readfds, &read_ready, sizeof (read_ready));

#else

  if (readfds) {
    for (i = 0; i < maxfdp1; i++) {
      if (FD_ISSET (i, readfds)) {
        count++;
      }
    }
  }

#endif

#ifdef DEBUG_SOCKET
  /* Debug info */
  DLOG ("Read sockets in readfds:");
  for (i = 0; i < MAX_FD; i++) {
    if (FD_ISSET (i, readfds)) {
      logger_printf ("  Socket %d ready for read\n", i);
    }
  }
#endif

  /* For write, assume all ready */
  if (writefds) {
    for (i = 0; i < maxfdp1; i++) {
      if (FD_ISSET (i, writefds)) {
        count++;
      }
    }
  }

  DLOG ("Select return: %d", count);

finish:
  unlock_kernel ();
  return count;
}

extern uint64 tsc_freq;         /* timestamp counter frequency */
static uint64 last_tsc = 0;
static struct timeval last_tp = {0, 0};

/* --!!-- This is a very crude implementation that assumes the first time
 * gettimeofday is called will be 1970... Let's read the real time from BIOS
 * during boot and setup an interrupt for the update later.
 *
 * We just want to make netperf work here, so...
 */
static int
sys_call_get_time (struct timeval *tp)
{
  uint64 cur_tsc;
  uint64 usec;
  uint64 sec;

  lock_kernel ();

  RDTSC (cur_tsc);

  usec = div64_64 (cur_tsc - last_tsc, tsc_freq);
  if (!usec) goto finish;
  usec = usec * 1000000LL;
  sec = div64_64 (usec, 1000000LL);
  usec = usec - sec * 1000000LL;

  last_tp.tv_sec += (long int) sec;
  last_tp.tv_usec += (long int) usec;

  last_tsc = cur_tsc;
finish:
  memcpy (tp, &last_tp, sizeof (struct timeval));
  
  unlock_kernel ();
  return 0;
}

static int
sys_call_get_sb_id ()
{
  return get_pcpu_id ();
}

static int
sys_call_getsockname (int sockfd, void *addr, void *len)
{
  quest_tss * tss;
  fd_table_entry_t fd_ent;
  task_id cur = percpu_read (current_task);
  struct tcp_pcb *tpcb;
  struct udp_pcb *upcb;

  if (!cur) {
    logger_printf ("No current task\n");
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    return -1;
  }

  DLOG ("getsockname on socket %d", sockfd);

  fd_ent = tss->fd_table[sockfd];

  switch (fd_ent.type) {
    case FD_TYPE_TCP :
      tpcb = (struct tcp_pcb *) fd_ent.entry;
      if (!tpcb) {
        DLOG ("getsockname socket %d not found", sockfd);
        return -1;
      }
      if (addr) {
        ((struct sockaddr_in *) addr)->sin_family = AF_INET;
        ((struct sockaddr_in *) addr)->sin_port = htons (tpcb->local_port);
        ((struct sockaddr_in *) addr)->sin_addr.s_addr = tpcb->local_ip.addr;
      }

      if (len)
        *((socklen_t *) len) = sizeof (struct sockaddr_in);
      break;
    case FD_TYPE_UDP :
      upcb = (struct udp_pcb *) fd_ent.entry;
      if (!upcb) {
        DLOG ("getsockname socket %d not found", sockfd);
        return -1;
      }
      if (addr) {
        ((struct sockaddr_in *) addr)->sin_family = AF_INET;
        ((struct sockaddr_in *) addr)->sin_port = htons (upcb->local_port);
        ((struct sockaddr_in *) addr)->sin_addr.s_addr = upcb->local_ip.addr;
      }

      if (len)
        *((socklen_t *) len) = sizeof (struct sockaddr_in);
      break;
    default :
      logger_printf ("Socket type %d not supported in getsockname\n",
                     fd_ent.type);
      return -1;
  }

  return 0;
}

static void
restart_networks_local ()
{
  cli ();
  {extern void r8169_reset (void); r8169_reset ();}
  {extern void r8169_free (void); r8169_free ();}
  {extern bool r8169_init (void); r8169_init ();}
  {extern bool r8169_register (void); r8169_register ();}
  {extern bool netsetup_init (void); netsetup_init ();}
  sti ();
}

static int
sys_call_recovery (int arg)
{
#ifdef USE_VMX
  uint32 cpu = get_pcpu_id ();
  struct netif * netif = NULL;
  struct ip_addr ipaddr, netmask, gw;

  switch (arg) {
    case 0:
      //cli ();
      shm->network_transmit_enabled[0] = FALSE;
      shm->network_transmit_enabled[1] = TRUE;
      //sti ();
      break;
    case 1:
      restart_networks_local ();
      break;
    case 3:
      netif = netif_find ("en0");
      netif_set_down (netif);
      logger_printf ("Interface en0 down in Sandbox %d!\n", cpu);
      if (netif) {
        IP4_ADDR(&ipaddr, 192, 168, 2, 11);
        IP4_ADDR(&netmask, 255, 255, 255, 0);
        IP4_ADDR(&gw, 192, 168, 2, 1);
        netif_set_addr (netif, &ipaddr, &netmask, &gw);
        netif_set_up (netif);
        logger_printf ("Interface en0 up in Sandbox %d!\n", cpu);
      } else {
        logger_printf ("Cannot find interface\n");
      }
      cli ();
      shm->network_transmit_enabled[1] = FALSE;
      sti ();
      logger_printf ("Hot Backup Set Up!\n");
      break;
    default:
      logger_printf ("Unknown recovery!\n");
  }
#endif
  return 0;
}

int sys_call_fcntl(int fd, int cmd, void* extra_arg)
{
  quest_tss * tss;
  task_id cur;
  int res;
  
  lock_kernel();

  cur = percpu_read (current_task);

  if (!cur) {
    logger_printf ("No current task\n");
    unlock_kernel();
    return -1;
  }

  tss = lookup_TSS (cur);

  if (tss == NULL) {
    logger_printf ("Task 0x%x does not exist\n", cur);
    unlock_kernel();
    return -1;
  }
  if(!tss->fd_table[fd].entry) {
    unlock_kernel();
    return -1;
  }
  
  switch(cmd) {
  case F_SETFL:
    tss->fd_table[fd].flags = *((int*)extra_arg);
    unlock_kernel();
    return 0;
    
  case F_GETFL:
    res = tss->fd_table[fd].flags;
    unlock_kernel();
    return res;
    
  default:
    DLOG("Unknown command sent to fcntl");
    unlock_kernel();
    return -1;
  }

  DLOG("Reached end of %s unexpectedly", __FUNCTION__);
  unlock_kernel();
  return -1;
}

int syscall_vshm_map(uint vshm_key, uint size, uint sandboxes, uint flags, void** addr)
{
#ifdef USE_VMX
  new_shared_memory_arena_msg_t msg;
  int pool_index;
  uint sandbox_num = 0;
  int pages_needed = DIV_ROUND_UP(size, 0x1000);
  void* arena_addr;
  uint permissions = flags & VSHM_ALL_ACCESS;

  if(!addr) return -1;

  if(vshm_key == 0) return -1;
  
  if(popcount(sandboxes) != 1) return -1;

  if(size == 0) return -1;
  
  while(!((1 << sandbox_num) & sandboxes) && (sandbox_num < SHM_MAX_SANDBOX)) sandbox_num++;

  if(sandbox_num >= SHM_MAX_SANDBOX || sandbox_num == get_pcpu_id()) return -1;

  if(pages_needed > POOL_SIZE_IN_PAGES) return -1;

  arena_addr = find_free_virtual_region(pages_needed * 0x1000);
  if(!arena_addr) {
    return -1;
  }
  
  lock_kernel();
  
  if(flags & VSHM_CREATE) {

    for(pool_index = 0; pool_index < NUM_POOLS_PER_SANDBOX; ++pool_index) {
      int i, j;
      for(i = 0 ; i < SHM_MAX_SANDBOX; ++i) {
        if(shm_comm.pools[i][pool_index].start_addr) {
          for(j = 0; j < POOL_SIZE_IN_PAGES; ++j) {
            if(shm_comm.pools[i][pool_index].keys_map[j] == vshm_key) {
              /* key is already in use */
              unlock_kernel();
              return -1;
            }
          }
        }
      }
    }
        
    for(pool_index = 0; pool_index < NUM_POOLS_PER_SANDBOX; ++pool_index) {
      if((shm_comm.pools[sandbox_num][pool_index].permissions == permissions) &&
         shm_comm.pools[sandbox_num][pool_index].start_addr &&
         ((POOL_SIZE_IN_PAGES - shm_comm.pools[sandbox_num][pool_index].used_pages) >= pages_needed) &&
         shm_comm.pools[sandbox_num][pool_index].created_locally) break;
    }
    
    if(pool_index < NUM_POOLS_PER_SANDBOX) { /* If we found a suitable pool already created */
      int i, j;
      int pages_allocated = 0;
      shm_pool_t* pool = &shm_comm.pools[sandbox_num][pool_index];
      uint bitmap = 0;
      
      for(i = 0; (pages_allocated < pages_needed) && (i < POOL_SIZE_IN_PAGES); ++i) {
        if(!pool->keys_map[i]) {
          bitmap |= (1 << i);
          pages_allocated++;
        }
      }
      
      if(i == POOL_SIZE_IN_PAGES) {
        /* This should never happen since we know we have enough free
           pages but just in case */
        unlock_kernel();
        return -1;
      }
      
      for(i = 0; i < POOL_SIZE_IN_PAGES; ++i) {
        if((1 << i) & bitmap) {
          pool->keys_map[i] = vshm_key;
        }
      }
      
      for(i = 0, j = 0; j < pages_needed; ++i) {
        if((1 << i) & bitmap) {
          if(!map_virtual_page_to_addr(7, ((i * 0x1000) + pool->start_addr) | 7,
                                       (j * 0x1000) + arena_addr)) {
            /* -- EM -- Need to unmap the pages already mapped, right
               now just panic */
            com1_printf("Failed to map address that should not have failed\n");
            panic("Failed to map address that should not have failed");
          }
          ++j;
        }
      }
      
      initialise_new_shared_memory_arena_msg(&msg, FALSE, vshm_key,
                                             pool_index,
                                             permissions, (phys_addr_t)NULL, bitmap);
    }
    else { /* Need to create a new pool */
      int i;
      shm_pool_t* pool = NULL;
      uint bitmap = (1 << pages_needed) - 1;
      for(pool_index = 0; pool_index < NUM_POOLS_PER_SANDBOX; ++pool_index) {
        if(!shm_comm.pools[sandbox_num][pool_index].start_addr) {
          /* Found free pool entry */
          pool = &shm_comm.pools[sandbox_num][pool_index];
          break;
        }
      }
      
      if(!pool) {
        /* Failed to find a free pool */
        unlock_kernel();
        return -1;
      }
      
      pool->created_locally = TRUE;
      pool->start_addr = shm_alloc_phys_frames_perm(POOL_SIZE_IN_PAGES, permissions);
      pool->permissions = permissions;
      
      if(pool->start_addr == 0xFFFFFFFF) {
        /* Failed to allocate pool setting start_addr back to zero to
           indicate that it is free */
        pool->start_addr = 0;
        unlock_kernel();
        return -1;
      }
      
      for(i = 0; i < pages_needed; ++i) {
        pool->keys_map[i] = vshm_key;
      }
      
      for(i = 0; i < pages_needed; ++i) {
        if(!map_virtual_page_to_addr(7, ((i * 0x1000) + pool->start_addr) | 7,
                                     (i * 0x1000) + arena_addr)) {
          /* -- EM -- Need to unmap the pages already mapped, right now
             just panic */
          com1_printf("Failed to map address that should not have failed\n");
          panic("Failed to map address that should not have failed");
        }
      }
      
      initialise_new_shared_memory_arena_msg(&msg, TRUE, vshm_key, pool_index,
                                             permissions, pool->start_addr,
                                             bitmap);
      
    }
    
    if(!send_intersandbox_msg(ISBM_NEW_SHARED_MEMORY_ARENA, sandbox_num, &msg, sizeof(msg))) {
      com1_printf("Failed to send ism, for vshm_map\n");
      panic("Failed to send ism, for vshm_map\n");
    }
    *addr = arena_addr;
  }
  else {
    int i, j;
    uint bitmap = 0;
    shm_pool_t* pool = NULL;
    uint matching_pages_count;
    for(pool_index = 0; pool_index < NUM_POOLS_PER_SANDBOX; ++pool_index) {
      if((shm_comm.pools[sandbox_num][pool_index].permissions == permissions) &&
         shm_comm.pools[sandbox_num][pool_index].start_addr &&
         (!shm_comm.pools[sandbox_num][pool_index].created_locally)) {
        int i;
        pool = &shm_comm.pools[sandbox_num][pool_index];
        for(i = 0; i < POOL_SIZE_IN_PAGES; ++i) {
          if(pool->keys_map[i] == vshm_key) {
            bitmap |= 1 << i;
            matching_pages_count++;
          }
        }
        break;
      }
    }
    
    if(matching_pages_count != pages_needed) {
      /* requested size does not match actual size, only care about
         page level granularity */
      unlock_kernel();
      return -1;
    }

    for(i = 0, j = 0; j < pages_needed; ++i) {
      if((1 << i) & bitmap) {
        if(!map_virtual_page_to_addr(7, ((i * 0x1000) + pool->start_addr) | 7,
                                     (j * 0x1000) + arena_addr)) {
          /* -- EM -- Need to unmap the pages already mapped, right
             now just panic */
          com1_printf("Failed to map address that should not have failed\n");
          panic("Failed to map address that should not have failed");
          }
        ++j;
      }
    }
    
  }
  unlock_kernel();
  return 0;
#else
  return -1;
#endif
}

sys_call_ptr_t _socket_syscall_table [] ALIGNED (0x1000) = {
  (sys_call_ptr_t) sys_call_open_socket,    /* 00 */
  (sys_call_ptr_t) sys_call_close,          /* 01 */
  (sys_call_ptr_t) sys_call_bind,           /* 02 */
  (sys_call_ptr_t) sys_call_connect,        /* 03 */
  (sys_call_ptr_t) sys_call_listen,         /* 04 */
  (sys_call_ptr_t) sys_call_accept,         /* 05 */
  (sys_call_ptr_t) sys_call_write,          /* 06 */
  (sys_call_ptr_t) sys_call_sendto,         /* 07 */
  (sys_call_ptr_t) sys_call_recv,           /* 08 */
  (sys_call_ptr_t) sys_call_select,         /* 09 */
  (sys_call_ptr_t) sys_call_get_time,       /* 10 */
  (sys_call_ptr_t) sys_call_get_sb_id,      /* 11 */
  (sys_call_ptr_t) sys_call_getsockname,    /* 12 */
  (sys_call_ptr_t) sys_call_recovery,       /* 13 */
  (sys_call_ptr_t) sys_call_fcntl,          /* 14 */
  (sys_call_ptr_t) syscall_vshm_map         /* 15 */
};

static bool socket_sys_call_initialized = FALSE;

/* socket layer initialization is done in netsetup.c */
void
socket_sys_call_init ()
{
  if (!socket_sys_call_initialized) {
    DLOG ("socket buffers initialized on sandbox %d", get_pcpu_id ());
    socket_sys_call_initialized = TRUE;
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
