#include "types.h"
#include "defs.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h"
#include "proc.h"

static struct spinlock netdevlock;

static int
netdev_read_locked(char *dst, int n)
{
  if (dst == 0 || n <= 0)
    return 0;

  for (;;) {
    if (proc && proc->killed)
      return -1;

    int r = net_rx((uchar*)dst, (uint)n);
    if (r != 0)
      return r;

    sleep(net_rx_chan(), &netdevlock);
  }
}

void
netdevinit(void)
{
  initlock(&netdevlock, "netdev");
  devsw[NETDEV].read = netdevread;
  devsw[NETDEV].write = netdevwrite;
}

int
netdevread(struct inode *ip, uint off, char *dst, int n)
{
  if (ip == 0)
    return -1;

  (void)off;
  iunlock(ip);
  acquire(&netdevlock);
  int r = netdev_read_locked(dst, n);
  release(&netdevlock);
  ilock(ip);
  return r;
}

int
netdevwrite(struct inode *ip, uint off, char *src, int n)
{
  if (ip == 0)
    return -1;

  (void)off;
  (void)src;
  (void)n;
  iunlock(ip);
  ilock(ip);
  return -1;
}
