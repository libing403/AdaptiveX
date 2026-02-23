
#include <conio.h> // 仅适用于Windows
#include<windows.h>

#include<stdlib.h>
#include "flexCore_if.h"
#include<stdio.h>
int main()
{

    adx_CoreInit(); //启动运动库
    int index=0;
    int enable=1;
    int startMode=0;
    int stopMode=0;
    int ret;
    for(int i=0;i<12;i++)
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
    
    double pos[3] = {0};
    double velocity = 100;
    double acc = 1000;
    double dec = 1000;
    double jerk = 1000;
    int direction=3;
    int coordSys = 0;
    int bufferMode = 1;
    int transitionMode=1;
    double transitionParameter[5]={0.0};
     
    //设置限位
    //x轴
    ret=adx_WriteBoolParameter(axisId[0], index, mcEnableLimitPos, true, 0);//启用软限位
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteBoolParameter(axisId[0], index, mcEnableLimitNeg, true, 0);//启用软限位
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteParameter(axisId[0], index, mcSWLimitPos, 100, 0);//设置正限位
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteParameter(axisId[0], index, mcSWLimitNeg, -10, 0);//设置负限位
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }
    //y轴
    ret=adx_WriteBoolParameter(axisId[1], index, mcEnableLimitPos, true, 0);
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteBoolParameter(axisId[1], index, mcEnableLimitNeg, true, 0);//启用软限位
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteParameter(axisId[1], index, mcSWLimitPos, 100, 0);
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }

    ret=adx_WriteParameter(axisId[1], index, mcSWLimitNeg, -10, 0);//设置负限位
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }
    //z轴
    ret=adx_WriteBoolParameter(axisId[2], index, mcEnableLimitPos, true, 0);
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteBoolParameter(axisId[2], index, mcEnableLimitNeg, true, 0);//启用软限位
    if (ret)
    {
        printf("adx_WriteBoolParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteParameter(axisId[2], index, mcSWLimitPos, 100, 0);
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }
    ret=adx_WriteParameter(axisId[2], index, mcSWLimitNeg, -10, 0);//设置负限位
    if (ret)
    {
        printf("adx_WriteParameter error %d\n", ret);
        return -1;
    }
    pos[0]=80;
    pos[1]=100;
    pos[2]=150;
    ret = adx_MoveLinearAbsolute(grpId, index, pos, velocity, acc, dec, jerk, direction,coordSys, bufferMode, transitionMode,transitionParameter);
    if (ret)
    {
        printf("adx_MoveLinearAbsolute error %d\n", ret);
        return -1;
    }

    pos[0]=20;
    pos[1]=-10;
    pos[2]=-50;
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
            if (ch == 'q'||ch=='Q') { // q键的ASCII码是113
                printf("adx_Core exit\n");
                break;
            }
        }        
    }
    adx_CoreClose();//关闭运动库
    return ret;
}