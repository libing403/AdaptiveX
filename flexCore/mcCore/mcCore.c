
/**
 * @file mcCore.c flexCore内核模块实现
 * @author li.bing(libinggalaxy@163.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-23
 * 
 * Copyright (c)  2025 li.bing
 * 
 */

#include <stdio.h>
#include "platformCfg.h"
#include "flexCore_if.h"
#include "ulog.h"
#include "axisControl.h"
#include "mcTypes.h"

void test_queue(void)
{
	ULOG_INFO("test_queue");
	CmdTrajectory_t pT1,*pT,*pTi;
	AxisGroupControl_t *group=&_adx_GRP[0];
	 
	for(int i=0;i<10;i++)
	{
		pT1.cmdNo = i;
		EnQueue(&group->trajectoryQ,&pT1);
	}
	pT=GetHeadPtr(&group->trajectoryQ);
	for(int i=0;i<10;i++)
	{
		pTi=pT+i;
		ULOG_INFO("pT->cmdNo=%d",pTi->cmdNo);
		 
	}
	for(int i=10;i<15;i++)
	{
		pT1.cmdNo = i;
		EnQueue(&group->trajectoryQ,&pT1);
	}
	for(int i=10;i<20;i++)
	{
		pTi=pT+i;
		ULOG_INFO("pT->cmdNo=%d",pTi->cmdNo);		 
	}
	ULOG_INFO("length=%d",QueueLength(&group->trajectoryQ));

}

void cmdParser(void)
{
    ULOG_INFO("cmdParser");
	Sleep(1000);
}

void motionPlan(void)
{
#if defined(WINDOWS_ENV)
	while(1)
#endif
	{

		int ret=0;
		ret=GroupControl_Update();
		if (ret)
		{
			ULOG_ERROR("GroupControl_Update error [ret=%d].", ret);
			//break;
		}
		//轴控制更新
		ret=axisControlUpdate();
		if(ret)
		{
			ULOG_ERROR("axisControlUpdate [ret=%d].", ret);
			//break;
		}
	

		Sleep(10);
	}	
}

void realTimeControl(void)
{

	//while(1)
	{
		int  ret=0;
		ret = GroupControl_RtUpdate();
		if (ret)
		{
			ULOG_ERROR("GroupControl_Update error [ret=%d].", ret);
			return;
		}	
		ret=axisControlRTUpdate();
		if(ret)
		{
			ULOG_ERROR("axisControlRTUpdate ret=%d.", ret);
			return;
		}

		globalReferenceTime=globalReferenceTime+CONTROL_PERIOD_SEC;
		
	}
}



