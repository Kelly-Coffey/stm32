#include "errno.h"
#include "stdio.h"
#include "string.h"
#include "sys/stat.h"

extern void putch(uint8_t c);

void _init(void)
{}

char *_sbrk_r(void *r, size_t incr)
{
  register char* stack_ptr asm ("sp");
  extern char    end asm ("end"); /* Defined by the linker.  */
  static char*   heap_end;
  char*          prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;
  
  prev_heap_end = heap_end;
  
  if (heap_end + incr > stack_ptr)
  {
      errno = ENOMEM;
      return (char *) -1;
  }
  
  heap_end += incr;

  return prev_heap_end;
}

_ssize_t _write_r(struct _reent *r, int fd, const void *buf, size_t cnt)
{
  const uint8_t *p = buf;
  for (size_t pos=0; pos<cnt; pos++) {
    if (*p == '\n') {
      putch('\r');
    }
    putch(*p++);
  }

  return cnt;
}

_ssize_t _read_r(void *r, int fd, void *buf, size_t cnt)
{
  return 0;
}

int _close_r(void *r, int fd)
{
  return 0;
}

off_t _lseek_r(void *r, int fd, off_t pos, int whence)
{
  return (off_t)0;
}

int _fstat_r(void *r, int fd, struct stat *st)
{
  st->st_mode = S_IFCHR;  
  return 0;
}

int _isatty(int fd)
{
  return 1;
}
