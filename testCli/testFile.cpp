
 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "flexCore_if.h"
#include "motorDrv.h"
#include "mcCore.h"
#include "testFun.h"
#include <windows.h>
#include "pipe.h"
#include <stdarg.h>
#include <string.h>


static HANDLE hPipe = NULL;

#define LOG_BUF_SIZE 512
void log_info(const char* fmt, ...)
{
    char buf[PIPE_BUF];
    DWORD written;
    va_list args;
    int len;

    if (g_hPipe == INVALID_HANDLE_VALUE)
        return;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
        return;

    if (len > (int)sizeof(buf))
        len = sizeof(buf);

    WriteFile(g_hPipe, buf, (DWORD)len, &written, NULL);
}

//DWORD WINAPI MonitorThread(LPVOID param)
int MonitorThread()
{
    int axis = 1;
    double pos = 0.0;
    
    //log_info("monitor thread created\n");
    //test_adx_ConnetModbus();
    //while (1) 
    {
        /* ===== 这里替换为真实查询逻辑 ===== */
        int ret=0;
        int axisId[4]={0,1,2,3};
        int axisNum=4;
        int index=0;
        double lgcPos[12]={0.0};
        double actPos[12]={0.0};
        int state[12] = {0};
        for(int i=0;i<axisNum;i++)
        {
            ret=adx_ReadMultiLogicalPos(axisId,index,axisNum,lgcPos);
            if(ret!=0)
            {
                log_info("adx_ReadLogicalPos error ret=%d\n",ret);
            }
            Sleep(5);
            ret=adx_ReadMultiActualPos(axisId,index,axisNum,actPos);
            if(ret!=0)
            {
                log_info("adx_ReadLogicalPos error ret=%d\n",ret);
            }
            Sleep(5);
            ret=adx_ReadMultiStatus(axisId, index, axisNum,&state[i]);
            if(ret!=0)
            {
                log_info("adx_ReadStatus error ret=%d\n",ret);
            }
            Sleep(5);
            
        }
        log_info("\n");
        log_info("logicalPos: %8.3f, %8.3f, %8.3f, %8.3f State: %d,%d,%d,%d\n",
             lgcPos[0], lgcPos[1], lgcPos[2], lgcPos[3],state[0],state[1],state[2],state[3]);
        log_info(" actualPos: %8.3f, %8.3f, %8.3f, %8.3f State: %d,%d,%d,%d\n",
             actPos[0], actPos[1], actPos[2],actPos[3], state[0],state[1],state[2],state[3]);    
        
    }

    //log_info("monitor thread exited\n");
    return 0;
}




/* ================== 主程序 ================== */
cmd_entry_t cmd_table[] =
{
    { "test_adx_ConnetModbus"            ,      test_adx_ConnetModbus           ,        "connect modbus test"          },
    { "test_adx_CoreInit"                ,      test_adx_CoreInit               ,        "initialize core"              },
    { "test_adx_CoreClose"               ,      test_adx_CoreClose              ,        "execute close test"           },
    { "test_adx_Power"                   ,      test_adx_Power                  ,        "power on/off test"            },
    { "test_adx_ReadMultiParameter"      ,      test_adx_ReadMultiParameter     ,        "read multi parameter test"    },
    { "test_adx_WriteMultiParameter"     ,      test_adx_WriteMultiParameter    ,        "write multi parameter test"   },
    { "test_adx_MoveAbsolute"            ,      test_adx_MoveAbsolute           ,        "move absolute test"           },
    { "test_adx_ReadTargetPos"           ,      test_adx_ReadTargetPos          ,        "read target position test"    },
    { "test_adx_ReadTargetVel"           ,      test_adx_ReadTargetVel          ,        "read target velocity test"    },
    { "test_adx_ReadLogicalPos"          ,      test_adx_ReadLogicalPos         ,        "read logical position test"   },
    { "test_adx_ReadActualPos"           ,      test_adx_ReadActualPos          ,        "read actual position test"    },
    { "test_adx_SetMotorScale"           ,      test_adx_SetMotorScale          ,        "set motor scale test"         },
    { "test_adx_SetGantryEnable"         ,      test_adx_SetGantryEnable        ,        "set gantry enable test"       },
    { "test_adx_SetGantryConfig"         ,      test_adx_SetGantryConfig        ,        "set gantry config test"       },
    { "test_adx_GantryMove"              ,      test_adx_GantryMove             ,        "gantry move test"             },
    { "help"                             ,      NULL                            ,        "show command list"            },
    { "exit"                             ,      NULL                            ,        "exit program"                 },
    { "quit"                             ,      NULL                            ,        "exit program"                 },
};

int maxCmdCount = sizeof(cmd_table) / sizeof(cmd_entry_t);


int main(void)
{

    STARTUPINFOA si = { sizeof(si) };
    PROCESS_INFORMATION pi;

    /* 创建主控制台 */
    
    AllocConsole();
    freopen("CONIN$", "r", stdin);
    freopen("CONOUT$", "w", stdout);

    printf("=== logger start ===\n");

    /* 启动日志窗口进程 */
    if (!CreateProcessA(
        "logger.exe",
        NULL,
        NULL,
        NULL,
        FALSE,
        CREATE_NEW_CONSOLE,
        NULL,
        NULL,
        &si,
        &pi))
    {
        printf("can not connnet  logger.exe\n");
        return -1;
    }

    Sleep(500); /* 等待日志窗口就绪 */

    /* 连接日志管道 */
    g_hPipe = CreateFileA(
        PIPE_NAME,
        GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (g_hPipe == INVALID_HANDLE_VALUE) {
        printf("can not connect to logger\n");
        return -1;
    }

    log_info("logger connected\n");

 

   // 初始化核心;
    int ret=adx_CoreInit();
    if(ret!=0)
    {
        printf("adx_CoreInit error ret=%d\n",ret);
        return 1;
    }
   //运动控制主循环线程
    HANDLE handle_motionPlan;
    handle_motionPlan = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)motionPlan, NULL, 0, NULL);
    printf("motion thread create ok!\n");
    // 设置线程优先级（相对于进程优先级）
    SetThreadPriority(handle_motionPlan, THREAD_PRIORITY_HIGHEST);
    
   //设置定时器
    DWORD_PTR dwUser = 0;
    MMRESULT timerID = timeSetEvent(10, 1, (LPTIMECALLBACK)realTimeControl, dwUser, TIME_PERIODIC);
    if (timerID != 0)
    printf("timer thread create ok!\n");
    char cmd[CMD_MAX_LEN];
    printf("=== C Test CLI ===\n");
    printf("Type 'help' to list commands\n");

    while (1)
    {
        //打印日志
        static int counter = 0;
        counter++;
        if(counter > 1000)
        {
            counter = 0;
            log_info("Executed command: %s", cmd);
        }

        printf(">> ");
        fflush(stdout);

        /* 读取一行命令 */
        if (fgets(cmd, sizeof(cmd), stdin) == NULL)
        {
            break;
        }

        /* 去掉换行符 */
        cmd[strcspn(cmd, "\r\n")] = 0;

        if (strlen(cmd) == 0)
        {
            MonitorThread();
            continue;
        }

        /* 处理 exit / quit */
        if (strcmp(cmd, "exit") == 0 ||
            strcmp(cmd, "quit") == 0 ||
            strcmp(cmd, "q") == 0)
        {
            printf("Exit program.\n");
            break;
        }

        /* 查表执行函数 */
        int found = 0;
       
        int cmd_count = sizeof(cmd_table) / sizeof(cmd_entry_t);
        for (int i = 0; i < cmd_count; i++)
        {
            if (strcmp(cmd, cmd_table[i].cmd) == 0)
            {
                found = 1;
                if (cmd_table[i].func)
                {
                    cmd_table[i].func();
                }
                else if (strcmp(cmd, "help") == 0)
                {
                    show_help();
                }
                break;
            }
        }

        if (!found)
        {
            printf("Unknown command: %s\n", cmd);
        }

    }

    return 0;
}


