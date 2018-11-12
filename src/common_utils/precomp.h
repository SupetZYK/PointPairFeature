#pragma once

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)
# define ZYK_EXPORTS  __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4
#  define ZYK_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define ZYK_EXPORTS
#endif