#ifdef DEBUG
#define TRACE my_printf
#else
#define TRACE
#endif

void my_printf(const char *fmt, ...);
