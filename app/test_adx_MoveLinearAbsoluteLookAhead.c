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
    ret=adx_GroupWriteParameter(grpId, index, mcSmoothLevel, 0, 0);
    if (ret)
    {
        printf("adx_GroupWriteParameter error %d\n", ret);
        return -1;
    }
    double pos[3] = {10, 20, 30};
    double velocity = 100;
    double acc = 1000;
    double dec = 1000;
    double jerk = 1000;
    int direction=3;
    int coordSys = 0;
    int bufferMode = 1;
    int transitionMode=1;
    double transitionParameter[5]={0.0};
    FILE *fp = fopen("testPath.txt", "r+");
    if (fp == NULL)
    {
        printf("open testPath error\n");
        return -1;
    }
    //读取路径文件
    int pathLength = 12;
    double path[100][3] = {0};
    int length = 0;
    for(int i=0;i<pathLength;i++)
    {
        length = fscanf(fp, "%lf %lf %lf", &path[i][0], &path[i][1], &path[i][2]);
        if (length != 3)
        {
            printf("read path error\n");
            fclose(fp);
            exit(-1);
        }
    }
    fclose(fp);
    //测试路径文件读取
    grpId = 1;
    for(int j=0;j<pathLength;j++)
    {
        pos[0] = path[j][0];
        pos[1] = path[j][1];
        pos[2] = path[j][2];
        index=j;
        ret = adx_MoveLinearAbsolute(grpId, index, pos, velocity, acc, dec, jerk, direction,coordSys, bufferMode, transitionMode,transitionParameter);
        if (ret)
        {
            printf("adx_MoveLinearAbsolute error %d\n", ret);
            return -1;
        }
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