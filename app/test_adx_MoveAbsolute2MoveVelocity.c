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
    double position=300;
    double velocity=150;
    double acceleration=500;
    double deceleration=500;
    double jerk=2000;
    int direction=3;
    int bufferMode=0;
    //0轴绝对定位到100
    ret=adx_MoveAbsolute(axisId, index, position, velocity, acceleration, deceleration,jerk, direction, bufferMode);
    if (ret)
    {
        printf("adx_MoveAbsolute error %d\n", ret);
        return ret;
    }
    Sleep(1000);
    velocity=300;
    jerk=2000;
    ret=adx_MoveVelocity(axisId, index, velocity, acceleration, deceleration,jerk,direction,bufferMode);
    if (ret)
    {
        printf("adx_MoveVelocity error %d\n", ret);
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