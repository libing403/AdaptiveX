#include <windows.h>
#include <stdio.h>
#include "pipe.h"

int main(void)
{
    HANDLE hPipe;
    char buf[PIPE_BUF];
    DWORD readBytes;

    /* 创建独立控制台 */
    AllocConsole();
    freopen("CONOUT$", "w", stdout);

    printf("=== logger start ===\n");

    /* 创建命名管道（只读） */
    hPipe = CreateNamedPipeA(
        PIPE_NAME,
        PIPE_ACCESS_INBOUND,
        PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
        1,
        PIPE_BUF,
        PIPE_BUF,
        0,
        NULL
    );

    if (hPipe == INVALID_HANDLE_VALUE) {
        printf("CreateNamedPipe failed\n");
        return -1;
    }

    printf("waiting for connection...\n");
    ConnectNamedPipe(hPipe, NULL);

    /* 接收并显示日志 */
    while (ReadFile(hPipe, buf, PIPE_BUF - 1, &readBytes, NULL)) {
        buf[readBytes] = '\0';
        printf("%s", buf);
        fflush(stdout);
    }

    CloseHandle(hPipe);
    return 0;
}


