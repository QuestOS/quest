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

/* TFTP "filesystem" driver.  TFTP is specified by RFC 1350. */

#include "lwip/ip.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "fs/filesys.h"
#include "mem/virtual.h"
#include "mem/physical.h"
#include "kernel.h"
#include "util/debug.h"
#include "util/circular.h"

//#define DEBUG_TFTP

#define TFTP_PORT 69
#define TFTP_BLOCK_SIZE 512
#define BLOCKS_PER_NODE 7       /* 7 * 512 + 8 < 4096 */
#define TFTP_RING_LEN 4

/*
 *         opcode  operation
 *           1     Read request (RRQ)
 *           2     Write request (WRQ)
 *           3     Data (DATA)
 *           4     Acknowledgment (ACK)
 *           5     Error (ERROR)
 */

enum {
  TFTP_OP_RRQ=1,
  TFTP_OP_WRQ,
  TFTP_OP_DATA,
  TFTP_OP_ACK,
  TFTP_OP_ERR
};

/*
 *           2 bytes     string    1 byte     string   1 byte
 *           ------------------------------------------------
 *          | Opcode |  Filename  |   0  |    Mode    |   0  |
 *           ------------------------------------------------
 *
 *                      Figure 5-1: RRQ/WRQ packet
 */

/*
 *                  2 bytes     2 bytes      n bytes
 *                  ----------------------------------
 *                 | Opcode |   Block #  |   Data     |
 *                  ----------------------------------
 *
 *                       Figure 5-2: DATA packet
 *
 */

/*
 *                        2 bytes     2 bytes
 *                        ---------------------
 *                       | Opcode |   Block #  |
 *                        ---------------------
 *
 *                        Figure 5-3: ACK packet
 */

#ifdef DEBUG_TFTP
#define DLOG(fmt,...) DLOG_PREFIX("tftp",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static struct netif *server_if;
static struct ip_addr server_ip;
static uint16 server_port = TFTP_PORT;
static struct udp_pcb *pcb;
static struct pbuf *incoming_buf[TFTP_RING_LEN];
static circular incoming;

struct _blocklist {
  struct _blocklist *next;
  uint32 len, start;
  uint8 blocks[TFTP_BLOCK_SIZE * BLOCKS_PER_NODE];
};
typedef struct _blocklist blocklist_t;

blocklist_t *curbuf, *curend;

/* format a read request */
static int
format_rrq (uint8 *buf, int len, const char *filename)
{
  int filename_len = strlen (filename);
  if (2+filename_len+1+6 > len)
    return 0;
  memset (buf, 0, len);
  buf[1] = TFTP_OP_RRQ;
  memcpy (buf+2, filename, filename_len);
  buf[2+filename_len] = 0;
  memcpy (buf+2+filename_len+1, "octet\0", 6); /* mode=octet */
  return 2+filename_len+1+6;
}

static int
send (uint8 *buf, uint32 len)
{
  struct pbuf *p;

  /* assume gateway has TFTP server */
  server_ip.addr = server_if->gw.addr;
  if (server_ip.addr == 0) {
    DLOG ("no server_ip");
    return -1;
  }

  p = pbuf_alloc (PBUF_TRANSPORT, len, PBUF_RAM);

  if (!p) {
    DLOG ("pbuf_alloc");
    return -1;
  }

  if (pbuf_take (p, buf, len) != ERR_OK) {
    pbuf_free (p);
    DLOG ("pbuf_take");
    return -1;
  }

  if (udp_sendto (pcb, p, &server_ip, server_port) != ERR_OK) {
    pbuf_free (p);
    DLOG ("udp_sendto");
    return -1;
  }

  pbuf_free (p);

  return len;
}

static blocklist_t *
alloc_node (void)
{
  uint32 frame = alloc_phys_frame ();
  if (frame == -1) return NULL;
  return map_virtual_page (frame | 3);
}

static void
free_node (blocklist_t *bl)
{
  uint32 frame = (uint32) get_phys_addr (bl);
  unmap_virtual_page (bl);
  free_phys_frame (frame);
}

#define NODE_CAPACITY (BLOCKS_PER_NODE * TFTP_BLOCK_SIZE)
static void
cache (uint8 *buf, uint32 len)
{
  while (len > 0) {
    if (curend && curend->len < NODE_CAPACITY) {
      /* can use existing node */
      uint32 rem = NODE_CAPACITY - curend->len;
      uint32 amount = len < rem ? len : rem;

      memcpy (&curend->blocks[curend->len], buf, amount);
      curend->len += amount;
      len -= amount;
      buf += amount;
    } else {
      /* need new node */
      blocklist_t *n = alloc_node ();
      memset (n, 0, sizeof (blocklist_t));
      n->next = NULL;
      n->len = len < NODE_CAPACITY ? len : NODE_CAPACITY;
      memcpy (n->blocks, buf, n->len);
      if (curend) {
        /* append to current end of list */
        curend->next = n;
        curend = curend->next;
      } else
        /* starting new list */
        curbuf = curend = n;
      len -= n->len;
      buf += n->len;
    }
  }
}

static void
free_cache (void)
{
  blocklist_t *bl;
  if (curbuf) {
    for (bl = curbuf; curbuf;) {
      bl = bl->next;
      free_node (curbuf);
      curbuf = bl;
    }
  }
  curend = curbuf = NULL;
}

int
eztftp_dir (char *pathname)
{
  struct pbuf *p, *q;
  uint8 buf[TFTP_BLOCK_SIZE+4], *ins;
  uint32 len, rem, filesize=0;

  circular_init (&incoming, incoming_buf,
                 TFTP_RING_LEN, sizeof (struct pbuf *));

  DLOG ("dir (%s)", pathname);
  if (pathname[0] == '/')
    /* some servers don't like leading slash */
    pathname++;

  if (curbuf) free_cache ();

  /* format and send a read request */
  len = format_rrq (buf, TFTP_BLOCK_SIZE+4, pathname);
  server_port = TFTP_PORT;
  if (send (buf, len) < 0) {
    DLOG ("failed to send request: %d %s", *((u16 *) buf), buf+2);
    return -1;
  }

  /* fetch file loop */
  rem = TFTP_BLOCK_SIZE+4; len=0; ins=buf;
  for (;;) {
    /* wait for incoming data */
    circular_remove (&incoming, &p);
    if (!p) continue;

    while (rem > 0) {
      /* copy data from current pbuf */
      memcpy (ins, p->payload, p->len);

      /* adjust counters */
      rem -= p->len;
      ins += p->len;
      len += p->len;

      /* chop pbuf off */
      q = p->next;
      if (q) pbuf_ref (q);
      pbuf_free (p);
      p = q;

      /* if last pbuf in chain */
      if (!p) break;
    }

    /* check for valid DATA packet */
    if (buf[1] == TFTP_OP_DATA) {
      DLOG ("received DATA packet num=0x%.04X len=%d bytes",
            (buf[2] << 8) | buf[3], len-4);
      /* send ACK the easy way */
      buf[1] = TFTP_OP_ACK;
      send (buf, 4);
      /* now put the data on our cached pbuf chain */
      cache (buf+4, len-4);
    } else if (buf[1] == TFTP_OP_ERR) {
      /* got error, probably file not found */
      DLOG ("error code=%d str=%s", (buf[2] << 8) | buf[3], &buf[4]);
      return -1;
    } else {
      /* discard buffer */
      DLOG ("received unexpected packet opcode=%d", buf[1]);
    }

    filesize += len-4;

    if (len - 4 < TFTP_BLOCK_SIZE)
      /* that was the last packet */
      break;

    /* reset buffer */
    rem = TFTP_BLOCK_SIZE+4; len=0; ins=buf;
  }

  DLOG ("opened file size=%d bytes", filesize);
  return filesize;
}

/* --YL-- Hack!!! Will be replaced by a new file system.
 *
 * eztftp_bulk_read is for loading Linux kernel image which is too
 * big for our kernel address space. An out-of-band memory region
 * "load_addr" should be provided. This will be replaced by a "decent"
 * file system in the very near future.
 */
int
eztftp_bulk_read (char *pathname, uint32 * load_addr)
{
  struct pbuf *p, *q;
  uint8 buf[TFTP_BLOCK_SIZE+4], *ins;
  uint32 len, rem, filesize=0;

  circular_init (&incoming, incoming_buf,
                 TFTP_RING_LEN, sizeof (struct pbuf *));

  DLOG ("dir (%s)", pathname);
  if (pathname[0] == '/')
    /* some servers don't like leading slash */
    pathname++;

  /* format and send a read request */
  len = format_rrq (buf, TFTP_BLOCK_SIZE+4, pathname);
  server_port = TFTP_PORT;
  if (send (buf, len) < 0) {
    DLOG ("failed to send request: %d %s", *((u16 *) buf), buf+2);
    return -1;
  }

  /* fetch file loop */
  rem = TFTP_BLOCK_SIZE+4; len=0; ins=buf;
  for (;;) {
    /* wait for incoming data */
    circular_remove (&incoming, &p);
    if (!p) continue;

    while (rem > 0) {
      /* copy data from current pbuf */
      memcpy (ins, p->payload, p->len);

      /* adjust counters */
      rem -= p->len;
      ins += p->len;
      len += p->len;

      /* chop pbuf off */
      q = p->next;
      if (q) pbuf_ref (q);
      pbuf_free (p);
      p = q;

      /* if last pbuf in chain */
      if (!p) break;
    }

    /* check for valid DATA packet */
    if (buf[1] == TFTP_OP_DATA) {
      DLOG ("received DATA packet num=0x%.04X len=%d bytes",
            (buf[2] << 8) | buf[3], len-4);
      /* send ACK the easy way */
      buf[1] = TFTP_OP_ACK;
      send (buf, 4);
      memcpy (load_addr, buf + 4, len - 4);
      load_addr = (uint32 *) (((uint8 *) load_addr) + len - 4);
    } else if (buf[1] == TFTP_OP_ERR) {
      /* got error, probably file not found */
      DLOG ("error code=%d str=%s", (buf[2] << 8) | buf[3], &buf[4]);
      return -1;
    } else {
      /* discard buffer */
      DLOG ("received unexpected packet opcode=%d", buf[1]);
    }

    filesize += len-4;

    if (len - 4 < TFTP_BLOCK_SIZE)
      /* that was the last packet */
      break;

    /* reset buffer */
    rem = TFTP_BLOCK_SIZE+4; len=0; ins=buf;
  }

  DLOG ("opened file size=%d bytes", filesize);
  return filesize;
}

int
eztftp_read (char *buf, int len)
{
  char *ptr = buf;
  int actual = 0;
  DLOG ("read (%p, %d)", buf, len);
  while (len > 0 && curbuf) {
    int amount = len < curbuf->len ? len : curbuf->len;

    /* copy data from current node */
    memcpy (ptr, &curbuf->blocks[curbuf->start], amount);
    ptr += amount;
    actual += amount;
    curbuf->start += amount;
    len -= amount;
    curbuf->len -= amount;

    /* check if current node is empty */
    if (curbuf->len <= 0) {
      blocklist_t *n = curbuf->next;
      free_node (curbuf);
      curbuf = n;
      if (curbuf == NULL)
        curend = NULL;
    }
  }

  return actual;
}

static void
recv_callback (void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, uint16 port)
{
  /* we get the pbuf to keep */
  DLOG ("recv_callback: addr=%p port=0x%.04X p->tot_len=%d bytes",
        addr->addr, port, p->tot_len);
  server_port = port;
  circular_insert_nowait (&incoming, &p);
}

bool
eztftp_mount (char *ifname)
{
  DLOG ("mount %s", ifname);
  circular_init (&incoming, incoming_buf,
                 TFTP_RING_LEN, sizeof (struct pbuf *));
  server_if = netif_find (ifname);
  if (server_if == NULL) return FALSE;
  server_ip.addr = 0;
  pcb = udp_new ();
  if (!pcb) return FALSE;
  if (udp_bind (pcb, IP_ADDR_ANY, 0) != ERR_OK)
    return FALSE;
  udp_recv (pcb, recv_callback, NULL);
  return TRUE;
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
