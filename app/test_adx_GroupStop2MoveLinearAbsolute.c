#include<stdlib.h>
#include "flexCore_if.h"
#include<stdio.h>
#include <conio.h> // 仅适用于Windows
#include<windows.h>
int main()
{

    adx_CoreInit(); //启动运动库
    int index=0;
    int enable=1;
    int startMode=0;
    int stopMode=0;
    int ret;
    for(int i=0;i<6;i++)
    {
         ret = adx_Power(i, enable, startMode,stopMode);
        if (ret)
        {
            printf("adx_Power error %d\n", ret);
            return -1;
        }
    }

    printf("test_adx_CreateAxisGroup start\n");
 
    int grpId=1;
    int axisId[3]={1,3,5};
  
    ret=adx_CreateAxisGroup(grpId,index,3,3,axisId);
    if (ret)
    {
        printf("adx_CreateAxisGroup error %d\n", ret);
        return -1;
    }
    ret = adx_GroupEnable(1);
    if (ret)
    {
        printf("adx_GroupEnable error %d", ret);
        return -1;
    }

    double pos[3] = {100, 200, 150};
    double velocity = 100;
    double acc = 200;
    double dec = 200;
    double jerk = 1000;
    int direction=3;
    int coordSys = 0;
    int bufferMode = 1;
    int transitionMode=1;
    double transitionParameter[5]={0.0};
    ret = adx_MoveLinearAbsolute(grpId, index, pos, velocity, acc, dec, jerk, direction,coordSys, bufferMode, transitionMode,transitionParameter);
    if (ret)
    {
        printf("adx_MoveLinearAbsolute error %d\n", ret);
        return -1;
    }
    Sleep(1500);
    ret=adx_GroupStop(grpId,index,dec,jerk,0);
    if (ret)
    {
        printf("adx_GroupStop error %d\n", ret);
        return -1;
    }
    Sleep(3000);
    pos[0] = 0;
    pos[1] = 200;   
    pos[2] = 100;
    ret = adx_MoveLinearAbsolute(grpId, index, pos, velocity, acc, dec, jerk, direction,coordSys, bufferMode, transitionMode,transitionParameter);
    if (ret)
    {
        printf("adx_MoveLinearAbsolute error %d\n", ret);
        return -1;
    }
 
    //等待运动执行完成
    while(1) {
        if (kbhit()) { // 检查是否有按键按下
            int ch = getch(); // 获取按键
            if (ch == 27) { // ESC键的ASCII码是27
                printf("adx_Core exit\n");
                break;
            }
        } 
        Sleep(10); // 每10毫秒检查一次
               
    }
    adx_CoreClose();//关闭运动库
    return ret;
}