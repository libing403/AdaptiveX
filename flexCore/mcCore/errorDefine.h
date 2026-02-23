#ifndef ERRORDEFINE_H
#define ERRORDEFINE_H

#ifdef __cplusplus
extern "C" {
#endif



enum ERRCODE{
    //指令执行错误
    CMDEXEC_SUCESS=0,
    SETPARAM_ERR,           //参数设置错误
    CMDDISPATCH_ERR,        //指令分发给状态机失败
    UNSUPPORTEDCMD_ERR,     //该状态下不可执行的指令或未知指令
    UNSUPPORTEDPARAM_ERR,   //该状态下不可使用的参数或不支持的参数
    //UNDEFINE_BEHAVIOR_ERR,  //未定义行为

    //控制器运行时错误
    CONTROLLER_ERROR=100,   //以下为控制器错误
    QUEUEINIT_ERR,          //命令队列初始化失败
    CREATE_MUTEX_ERR,       //创建互斥量失败
    SPEEDRATEINIT_ERR,      //速度率初始化失败
    QUEUEWRITE_ERR,         //写队列出错

};



#ifdef __cplusplus
}  
#endif     
#endif // !ERRORDEFINE_H