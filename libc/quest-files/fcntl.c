#include <fcntl.h>
#include <stdarg.h>

int fcntl(int fd, int cmd, ... /* arg */ )
{
  va_list vl;
  int arg_int;

  switch(cmd) {
    
  case F_SETFL:
    
    va_start(vl, cmd);
    arg_int = va_arg(vl, int);
    va_end(vl);
    return syscall_fcntl(fd, cmd, &arg_int);
    
  case F_GETFL:

    return syscall_fcntl(fd, cmd, NULL);
    
  default:

    return -1;
  }
}
