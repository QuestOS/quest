#undef TARGET_OS_CPP_BUILTINS
#define TARGET_OS_CPP_BUILTINS()	\
  do {					\
    builtin_define_std ("quest");	\
    builtin_define_std ("unix");	\
    builtin_assert ("system=quest");	\
    builtin_assert ("system=unix");	\
  } while(0);


/* note that adding these two lines cause an error in gcc-4.7.0 the
   build process works fine without them until someone can work out an
   alternative */

/*
#undef TARGET_VERSION
#define TARGET_VERSION fprintf(stderr, " (i386 quest)");  
*/
