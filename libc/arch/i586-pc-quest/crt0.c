
extern int main(); //int argc, char **argv, char **environ);

extern char __bss_start, _end; // BSS should be the last think before _end

// XXX: environment
char *__env[1] = { 0 };
//extern char **environ = __env;

void _start(int argc, char *argv[]) {
  char *i;

  // zero BSS
  for(i = &__bss_start; i < &_end; i++){
    *i = 0; 
  } 

  asm volatile("finit");

  // XXX: get argc and argv

  exit(main(argc, argv,  __env));
}
