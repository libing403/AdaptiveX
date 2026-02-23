#include<stdlib.h>
#include "flexCore_if.h"
#include<stdio.h>
#include <conio.h> // 仅适用于Windows
#include<windows.h>
int main()
{

    adx_CoreInit(); //启动运动库
    int axisId=0;
    int enable=1;
    int startMode=0;
    int stopMode=0;
    int ret = adx_Power(axisId, enable, startMode,stopMode);//启用0轴
    if (ret)
    {
        printf("adx_Power error %d\n", ret);
        return ret;
    }
    int index=0;
    double position=200;
    double velocity=50;
    double acceleration=100;
    double deceleration=100;
    double jerk=0.0;
    int direction=3;
    int bufferMode=0;
    //0轴绝对定位到100
    ret=adx_MoveAbsolute(axisId, index, position, velocity, acceleration, deceleration,jerk, direction, bufferMode);
    if (ret)
    {
        printf("adx_MoveAbsolute error %d\n", ret);
        return ret;
    }
    Sleep(1500);
    double velFactor=0.5;
    double accFactor=1.0;
    double jerkFactor=1.0;
    bufferMode=1; //0-缓冲模式；1-立即执行
    ret=adx_SetOverride(axisId, index, velFactor, accFactor, jerkFactor,bufferMode);
    if (ret)
    {
        printf("adx_SetOverride error %d\n", ret);
        return ret;
    }
    Sleep(1500);
    velFactor=1; //速度倍率
    ret=adx_SetOverride(axisId, index, velFactor, accFactor, jerkFactor, bufferMode);
    if (ret)
    {
        printf("adx_SetOverride error %d\n", ret);
        return ret;
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
    }
    adx_CoreClose();//关闭运动库
    return ret;
}