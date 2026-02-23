#ifndef PIPE_H
#define PIPE_H

#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
 
static HANDLE g_hPipe = INVALID_HANDLE_VALUE;

#define PIPE_NAME "\\\\.\\pipe\\DualConsolePipe"
#define PIPE_BUF  512

 

#endif
