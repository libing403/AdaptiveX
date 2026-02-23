

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "flexCore_if.h"
#include "cmdNo.h"
//第三方库
#include"ulog.h"




/****************************************************************
 * 工具函数：从 buf[] 中解析 Modbus 数据
 * 支持大小端模式：
 *   mode = 0 → big-endian  (高位字在前)
 *   mode = 1 → little-endian（低位字在前）
 ****************************************************************/

#include <stdint.h>
#include <string.h>

typedef enum {
    ENDIAN_BIG,        // AB CD -> float ABCD (标准Modbus)
    ENDIAN_LITTLE,     // CD AB -> float ABCD
    ENDIAN_WORD_SWAP,  // CD AB -> float ABCD
    ENDIAN_BYTE_SWAP,   // BA DC -> float ABCD
    ENDIAN_BYTEWORD_SWAP // DC BA-> float ABCD

} ENDIAN_MODE;

void Modbus_WriteInt16(unsigned char *buf, int regIndex, int16_t value, ENDIAN_MODE mode)
{
    unsigned char bytes[2];
    memcpy(bytes, &value, 2);

    switch (mode)
    {
        case ENDIAN_BIG:   // AB
            buf[regIndex*2]     = bytes[0];
            buf[regIndex*2 + 1] = bytes[1];
            break;

        default:           // BA
            buf[regIndex*2]     = bytes[1];
            buf[regIndex*2 + 1] = bytes[0];
            break;
    }
}


void Modbus_WriteUInt16(unsigned char *buf, int regIndex, uint16_t value, ENDIAN_MODE mode)
{
    unsigned char bytes[2];
    memcpy(bytes, &value, 2);

    if (mode == ENDIAN_BIG)
    {
        buf[regIndex*2]     = bytes[0];
        buf[regIndex*2 + 1] = bytes[1];
    }
    else
    {
        buf[regIndex*2]     = bytes[1];
        buf[regIndex*2 + 1] = bytes[0];
    }
}

int16_t Modbus_ReadInt16(unsigned char *buf, int regIndex, ENDIAN_MODE mode)
{
    unsigned char bytes[2];

    if (mode == ENDIAN_BIG)
    {
        bytes[0] = buf[regIndex*2];
        bytes[1] = buf[regIndex*2 + 1];
    }
    else
    {
        bytes[0] = buf[regIndex*2 + 1];
        bytes[1] = buf[regIndex*2];
    }

    int16_t value;
    memcpy(&value, bytes, 2);
    return value;
}

uint16_t Modbus_ReadUInt16(unsigned char *buf, int regIndex, ENDIAN_MODE mode)
{
    unsigned char bytes[2];

    if (mode == ENDIAN_BIG)
    {
        bytes[0] = buf[regIndex*2];
        bytes[1] = buf[regIndex*2 + 1];
    }
    else
    {
        bytes[0] = buf[regIndex*2 + 1];
        bytes[1] = buf[regIndex*2];
    }

    uint16_t value;
    memcpy(&value, bytes, 2);
    return value;
}


float Modbus_ReadFloat(unsigned char *buf, int regIndex, ENDIAN_MODE mode)
{
    unsigned char bytes[4];

    // buf: 每个寄存器存2字节，即 buf[regIndex*2]
    unsigned char A = buf[regIndex*2];
    unsigned char B = buf[regIndex*2 + 1];
    unsigned char C = buf[regIndex*2 + 2];
    unsigned char D = buf[regIndex*2 + 3];

    switch (mode)
    {
        case ENDIAN_BIG: // AB CD
            bytes[0] = A; bytes[1] = B; bytes[2] = C; bytes[3] = D;
            break;

        case ENDIAN_LITTLE: // CD AB
            bytes[0] = C; bytes[1] = D; bytes[2] = A; bytes[3] = B;
            break;

        case ENDIAN_WORD_SWAP: // CD AB
            bytes[0] = C; bytes[1] = D; bytes[2] = A; bytes[3] = B;
            break;

        case ENDIAN_BYTE_SWAP: // BA DC
            bytes[0] = B; bytes[1] = A; bytes[2] = D; bytes[3] = C;
            break;
        case ENDIAN_BYTEWORD_SWAP:
            bytes[0] = D; bytes[1] = C; bytes[2] = B; bytes[3] = A;
            break;
    }

    float result;
    memcpy(&result, bytes, 4);
    return result;
}

int32_t Modbus_ReadInt32(unsigned char *buf, int regIndex, ENDIAN_MODE mode)
{
    unsigned char bytes[4];

    unsigned char A = buf[regIndex*2];
    unsigned char B = buf[regIndex*2 + 1];
    unsigned char C = buf[regIndex*2 + 2];
    unsigned char D = buf[regIndex*2 + 3];

    switch(mode)
    {
        case ENDIAN_BIG:        
          bytes[0] = A; bytes[1] = B; bytes[2] = C; bytes[3] = D; 
          break;
        case ENDIAN_LITTLE:     
          bytes[0] = C; bytes[1] = D; bytes[2] = A; bytes[3] = B; 
          break;
        case ENDIAN_WORD_SWAP: 
           bytes[0] = C; bytes[1] = D; bytes[2] = A; bytes[3] = B; 
           break;
        case ENDIAN_BYTE_SWAP:  
          bytes[0] = B; bytes[1] = A; bytes[2] = D; bytes[3] = C; 
          break;
        case ENDIAN_BYTEWORD_SWAP:
          bytes[0] = D; bytes[1] = C; bytes[2] = B; bytes[3] = A;
          break;
    }

    int32_t v;
    memcpy(&v, bytes, 4);
    return v;
}


double Modbus_ReadDouble(unsigned char *buf, int regIndex, ENDIAN_MODE mode)
{
    unsigned char bytes[8];

    // 4 registers = 8 bytes
    unsigned char R[8];
    for (int i = 0; i < 8; i++)
        R[i] = buf[regIndex*2 + i];
    switch (mode)
    {
        case ENDIAN_BIG:  // ABCD EFGH
            memcpy(bytes, R, 8);
            break;

        case ENDIAN_LITTLE: // EFGH ABCD
            bytes[0] = R[4]; bytes[1] = R[5]; bytes[2] = R[6]; bytes[3] = R[7];
            bytes[4] = R[0]; bytes[5] = R[1]; bytes[6] = R[2]; bytes[7] = R[3];
            break;

        case ENDIAN_WORD_SWAP: // GH EF CD AB
            bytes[0] = R[6]; bytes[1] = R[7];
            bytes[2] = R[4]; bytes[3] = R[5];
            bytes[4] = R[2]; bytes[5] = R[3];
            bytes[6] = R[0]; bytes[7] = R[1];
            break;

        case ENDIAN_BYTE_SWAP: // BA DC FE HG
            for (int i = 0; i < 4; i++) {
                bytes[i*2]   = R[i*2+1];
                bytes[i*2+1] = R[i*2];
            }
            break;
        case ENDIAN_BYTEWORD_SWAP:  //HG FE DC BA
          bytes[0] = R[7]; bytes[1] = R[6]; bytes[2] = R[5]; bytes[3] = R[4];
          bytes[4] = R[3]; bytes[5] = R[2]; bytes[6] = R[1]; bytes[7] = R[0];
          break;

    }

    double res;
    memcpy(&res, bytes, 8);
    return res;
}



void Modbus_WriteFloat(unsigned char *buf, int regIndex, float value, ENDIAN_MODE mode)
{
    unsigned char bytes[4];
    memcpy(bytes, &value, 4);
    switch (mode)
    {
        case ENDIAN_BIG:  // AB CD
            buf[regIndex*2]     = bytes[0];
            buf[regIndex*2 + 1] = bytes[1];
            buf[regIndex*2 + 2] = bytes[2];
            buf[regIndex*2 + 3] = bytes[3];
            break;
        case ENDIAN_LITTLE: // CD AB
        case ENDIAN_WORD_SWAP:
            buf[regIndex*2]     = bytes[2];
            buf[regIndex*2 + 1] = bytes[3];
            buf[regIndex*2 + 2] = bytes[0];
            buf[regIndex*2 + 3] = bytes[1];
            break;
        case ENDIAN_BYTE_SWAP: // BA DC
            buf[regIndex*2]     = bytes[1];
            buf[regIndex*2 + 1] = bytes[0];
            buf[regIndex*2 + 2] = bytes[3];
            buf[regIndex*2 + 3] = bytes[2];
            break;
        case ENDIAN_BYTEWORD_SWAP:
            buf[regIndex*2]     = bytes[3];
            buf[regIndex*2 + 1] = bytes[2];
            buf[regIndex*2 + 2] = bytes[1];
            buf[regIndex*2 + 3] = bytes[0];
            break;
    }
}



void Modbus_WriteInt32(unsigned char *buf, int regIndex, int32_t value, ENDIAN_MODE mode)
{
    unsigned char bytes[4];
    memcpy(bytes, &value, 4);

    switch (mode)
    {
        case ENDIAN_BIG:  // AB CD
            buf[regIndex*2]     = bytes[0];
            buf[regIndex*2 + 1] = bytes[1];
            buf[regIndex*2 + 2] = bytes[2];
            buf[regIndex*2 + 3] = bytes[3];
            break;

        case ENDIAN_LITTLE: // CD AB
        case ENDIAN_WORD_SWAP:
            buf[regIndex*2]     = bytes[2];
            buf[regIndex*2 + 1] = bytes[3];
            buf[regIndex*2 + 2] = bytes[0];
            buf[regIndex*2 + 3] = bytes[1];
            break;

        case ENDIAN_BYTE_SWAP: // BA DC
            buf[regIndex*2]     = bytes[1];
            buf[regIndex*2 + 1] = bytes[0];
            buf[regIndex*2 + 2] = bytes[3];
            buf[regIndex*2 + 3] = bytes[2];
            break;

        case ENDIAN_BYTEWORD_SWAP:
            buf[regIndex*2]     = bytes[3];
            buf[regIndex*2 + 1] = bytes[2];
            buf[regIndex*2 + 2] = bytes[1];
            buf[regIndex*2 + 3] = bytes[0];
            break;
    }
}


void Modbus_WriteDouble(unsigned char *buf, int regIndex, double value, ENDIAN_MODE mode)
{
    unsigned char bytes[8];
    memcpy(bytes, &value, 8);
    switch (mode)
    {
        case ENDIAN_BIG:
            for (int i = 0; i < 8; i++)
                buf[regIndex*2 + i] = bytes[i];
            break;

        case ENDIAN_LITTLE:  // 交换 4 字节字序
            buf[regIndex*2]     = bytes[4];
            buf[regIndex*2 + 1] = bytes[5];
            buf[regIndex*2 + 2] = bytes[6];
            buf[regIndex*2 + 3] = bytes[7];
            buf[regIndex*2 + 4] = bytes[0];
            buf[regIndex*2 + 5] = bytes[1];
            buf[regIndex*2 + 6] = bytes[2];
            buf[regIndex*2 + 7] = bytes[3];
            break;

        case ENDIAN_WORD_SWAP:  // GH EF CD AB（每16位字交换）
            buf[regIndex*2]     = bytes[6];
            buf[regIndex*2 + 1] = bytes[7];
            buf[regIndex*2 + 2] = bytes[4];
            buf[regIndex*2 + 3] = bytes[5];
            buf[regIndex*2 + 4] = bytes[2];
            buf[regIndex*2 + 5] = bytes[3];
            buf[regIndex*2 + 6] = bytes[0];
            buf[regIndex*2 + 7] = bytes[1];
            break;

        case ENDIAN_BYTE_SWAP:  // 每字节交换 BA DC FE HG
            for (int i = 0; i < 4; i++)
            {
                buf[regIndex*2 + i*2]     = bytes[i*2 + 1];
                buf[regIndex*2 + i*2 + 1] = bytes[i*2];
            }
            break;

        case ENDIAN_BYTEWORD_SWAP:
            buf[regIndex*2]     = bytes[7];
            buf[regIndex*2 + 1] = bytes[6];
            buf[regIndex*2 + 2] = bytes[5];
            buf[regIndex*2 + 3] = bytes[4];
            buf[regIndex*2 + 4] = bytes[3];
            buf[regIndex*2 + 5] = bytes[2];
            buf[regIndex*2 + 6] = bytes[1];
            buf[regIndex*2 + 7] = bytes[0];
            break;
    }
}






/********************************************************************
 *                    MAIN EXECUTION FUNCTION
 ********************************************************************/
typedef enum
{
    MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE                /*!< Update register values. */
} eMBRegisterMode;

#define cmd_adx_CoreInit_ADDR                   100
#define cmd_adx_CoreClose_ADDR                  108
#define cmd_adx_Power_ADDR                      116
#define cmd_adx_WriteMultiParam_ADDR            124
#define cmd_adx_MoveAbsolute_ADDR               244
#define cmd_adx_MoveRelative_ADDR               264
#define cmd_adx_SetOverride_ADDR                284
#define cmd_adx_MoveVelocity_ADDR               300
#define cmd_adx_Halt_ADDR                       316
#define cmd_adx_Stop_ADDR                       332
#define cmd_adx_GroupDisable_ADDR               348
#define cmd_adx_GroupEnable_ADDR                356
#define cmd_adx_AddAxisToGroup_ADDR             364
#define cmd_adx_CreateAxisGroup_ADDR            372
#define cmd_adx_SetKinematics_ADDR              492
#define cmd_adx_MoveLinearAbsolute_ADDR         508
#define cmd_adx_GroupWriteMultiParam_ADDR       572
#define cmd_adx_GroupSetOverride_ADDR           692
#define cmd_adx_GroupInterrupt_ADDR             708
#define cmd_adx_GroupContinue_ADDR              724
#define cmd_adx_GroupStop_ADDR                  740
#define cmd_adx_GroupSetMcode_ADDR              756
#define cmd_adx_RegisterSetDeviceFun_ADDR       772
#define cmd_adx_RegisterReadDeviceFun_ADDR      788
#define cmd_adx_SetDeviceStatus_ADDR            804
#define cmd_adx_ReadDeviceStatus_ADDR           10000
#define cmd_adx_ReadMultiParam_ADDR             10016
#define cmd_adx_GroupReadMultiParam_ADDR        13856
#define cmd_adx_ReadTargetPos_ADDR              20000                  
#define cmd_adx_ReadTargetVel_ADDR              20064
#define cmd_adx_ReadTargetAcc_ADDR              20128
#define cmd_adx_ReadTargetJerk_ADDR             20192
#define cmd_adx_ReadLogicalPos_ADDR             20256
#define cmd_adx_ReadLogicalVel_ADDR             20320
#define cmd_adx_ReadLogicalAcc_ADDR             20384
#define cmd_adx_ReadLogicalJerk_ADDR            20448
#define cmd_adx_ReadActualPos_ADDR              20512
#define cmd_adx_ReadActualVel_ADDR              20576
#define cmd_adx_ReadActualAcc_ADDR              20640
#define cmd_adx_ReadActualJerk_ADDR             20704
#define cmd_adx_ReadStatus_ADDR                 20768
#define cmd_adx_GroupReadTargetPos_ADDR         20832
#define cmd_adx_GroupReadLogicalPos_ADDR        20880
#define cmd_adx_GroupReadLogicalVel_ADDR        20928
#define cmd_adx_GroupReadLogicalAcc_ADDR        20976
#define cmd_adx_GroupReadLogicalJerk_ADDR       21024
#define cmd_adx_GroupReadSpeed_ADDR             21072
#define cmd_adx_GroupReadStatus_ADDR            21080
#define cmd_adx_ReadFloatParam_ADDR             21088
#define cmd_adx_ReadIntParam_ADDR               27488
#define cmd_adx_GroupReadFloatParam_ADDR        33888
#define cmd_adx_GroupReadIntParam_ADDR          34688
#define cmd_adx_MaxCmdNum_ADDR                  35488




//地址偏移定义
#define AXIS_MULTIPARAM_ADDR_OFFSET        120      //轴参数结构体偏移地址
#define GROUP_MULTIPARAM_ADDR_OFFSET       120      //轴组参数结构体偏移地址

#define AXIS_PARAM_ADDR_OFFSET             6400    //轴参数偏移地址，32个轴
#define GROUP_PARAM_ADDR_OFFSET            6400      //轴组参数偏移地址,4个轴组


//参数id定义
typedef enum
{
    AXIS_TARGET_POS          = 0    ,    //轴浮点型参数id定义
    AXIS_LOGICAL_POS                ,
    AXIS_LOGICAL_VEL                ,
    AXIS_LOGICAL_ACC                ,
    AXIS_LOGICAL_JERK               ,
    AXIS_ACTUAL_POS                 ,
    AXIS_ACTUAL_VEL                 ,
    AXIS_ACTUAL_ACC                 ,
    AXIS_STATUS              = 3200 ,     //轴整型参数id定义
    AXIS_LMTEABLE                   ,
    GROUP_STATUS             = 6800 ,      //轴组整型参数id定义
    GROUP_LMTEABLE            
} ParamId_enum;



typedef struct
{
    int cmdNo;    // 命令编号
    int axisId;//轴组或轴ID
    int paramId;//参数ID
    int addr;  // Modbus register address
    int size;       // 寄存器个数
} CmdParamMap_t;


typedef enum
{
	INVALID_ADDR_ERR     = 1000,
	CMD_NOT_FOUND_ERR    = 1001,
    AXIS_NOT_FOUND_ERR   = 1002,
    GROUP_NOT_FOUND_ERR  = 1003,
	EXECUTE_CMD_ERR      = 1004,
} CmdExecuteError_enum;

int excuteWriteCmd(unsigned char* pucRegBuffer, int usAddress,  int mbMode);
int excuteReadCmd(unsigned char* pucRegBuffer, int usAddress,  int mbMode);
int cmdExecute( unsigned char *pucRegBuffer,int usAddress, int usNRegs,int mbMode)
{
    int ret = 0;
   // ENDIAN_MODE endianMode = ENDIAN_BYTEWORD_SWAP;
   printf("pucRegBuffer: usAddress %d  data: ",usAddress);
   for(int i=0;i<usNRegs*2;i++)
   {
    printf("%x ",pucRegBuffer[i]);
   }
    printf("\n ");

    int address = usAddress - 1;
    if (address < 100 || address>cmd_adx_MaxCmdNum_ADDR)
    {
            ULOG_ERROR("cmdExecute: Invalid address=%d", usAddress);
            return INVALID_ADDR_ERR;
    }
    else if (address < cmd_adx_ReadDeviceStatus_ADDR)
    {
        //写命令
        ret = excuteWriteCmd(pucRegBuffer, usAddress,  mbMode);
        if (ret)
        {
            ULOG_ERROR("cmdExecute: excuteWriteCmd error ret=%d", ret);
            return ret;
        }
    }
    else if ( address < cmd_adx_MaxCmdNum_ADDR)
    {
        //读命令
        ret = excuteReadCmd(pucRegBuffer, usAddress, mbMode);
        if (ret)
        {
            ULOG_ERROR("cmdExecute: excuteReadCmd error ret=%d", ret);
            return ret;
        }
    }
    else
    {
        ULOG_ERROR("cmdExecute: Command not found address=%d", usAddress);
        return CMD_NOT_FOUND_ERR;
    }
    return 0;
}


int excuteWriteCmd(unsigned char* pucRegBuffer, int usAddress,  int mbMode)
{
    (void)mbMode;
    int ret = 0;  
    ENDIAN_MODE endianMode = ENDIAN_BYTEWORD_SWAP;
    //初始化轴控制器
    switch (usAddress - 1)
    {

    case cmd_adx_CoreInit_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_CoreInit\n");
        int idx = 0;
        int a = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        float b = Modbus_ReadFloat(&pucRegBuffer[idx], 0, endianMode);
        printf("a=%d,b=%f\n", a, b);
        ret = adx_CoreInit();
        if (ret)
            ULOG_ERROR("adx_CoreInit error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_CoreInit finish\n");
        break;
    }

    case cmd_adx_CoreClose_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_CoreClose\n");
        int idx = 0;
        int a = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        float b = Modbus_ReadFloat(&pucRegBuffer[idx], 0, endianMode);
        printf("a=%d,b=%f\n", a, b);
        ret = adx_CoreClose();
        if (ret)
            ULOG_ERROR("adx_CoreClose error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_CoreClose finish\n");
        break;
    }

    //====================================================
    case cmd_adx_Power_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_Power\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int enable = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        int startMode = Modbus_ReadInt32(&pucRegBuffer[0], 4, endianMode);
        int stopMode = Modbus_ReadInt32(&pucRegBuffer[0], 6, endianMode);
        ret = adx_Power(axisId, enable, startMode, stopMode);
        if (ret)
            ULOG_ERROR("adx_Power error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_Power finish\n");
        break;
    }

    case cmd_adx_WriteMultiParam_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_WriteMultiParam\n");
        int idx = 0;
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[4], 0, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[8], 0, endianMode);
        axisParam_t param;
        idx = 12;
        param.ppr = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.dirPol = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.speed = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.acc = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.dpr = Modbus_ReadFloat(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.org = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.mdis = Modbus_ReadFloat(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.pdis = Modbus_ReadFloat(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.resetDir = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        idx = idx + 4;
        param.resetSpeed = Modbus_ReadInt32(&pucRegBuffer[idx], 0, endianMode);
        printf("adx_WriteMultiParameter:\n"
            "axisId=%d,\n"
            "index=%d,\n"
            "buffermode=%d,\n"
            "ppr=%u,\n"
            "dirPol=%d,\n"
            "speed=%u,\n"
            "acc=%u,\n"
            "dpr=%.3f,\n"
            "org=%d,\n"
            "mdis=%.3f,\n"
            "pdis=%.3f,\n"
            "resetDir=%d,\n"
            "resetSpeed=%u\n",
            axisId, index, bufferMode, param.ppr, param.dirPol, param.speed, param.acc, param.dpr, param.org,
            param.mdis, param.pdis, param.resetDir, param.resetSpeed);
        ret = adx_WriteMultiParameters(axisId, index, &param, bufferMode);
        if (ret)
            ULOG_ERROR("adx_WriteMultiParameters error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_WriteMultiParam finish\n");
        break;
    }
    case cmd_adx_MoveAbsolute_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_MoveAbsolute\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        double pos = Modbus_ReadFloat(&pucRegBuffer[0], 4, endianMode);
        double vel = Modbus_ReadFloat(&pucRegBuffer[0], 6, endianMode);
        double acc = Modbus_ReadFloat(&pucRegBuffer[0], 8, endianMode);
        double dec = Modbus_ReadFloat(&pucRegBuffer[0], 10, endianMode);
        double jerk = Modbus_ReadFloat(&pucRegBuffer[0], 12, endianMode);
        int direction = Modbus_ReadInt32(&pucRegBuffer[0], 14, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[0], 16, endianMode);
        ret = adx_MoveAbsolute(axisId, index, pos, vel, acc, dec, jerk, direction, bufferMode);
        if (ret)
            ULOG_ERROR("adx_MoveAbsolute error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_MoveAbsolute finish\n");
        break;
    }
    case cmd_adx_MoveRelative_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_MoveRelative\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        double pos = Modbus_ReadFloat(&pucRegBuffer[0], 4, endianMode);
        double vel = Modbus_ReadFloat(&pucRegBuffer[0], 6, endianMode);
        double acc = Modbus_ReadFloat(&pucRegBuffer[0], 8, endianMode);
        double dec = Modbus_ReadFloat(&pucRegBuffer[0], 10, endianMode);
        double jerk = Modbus_ReadFloat(&pucRegBuffer[0], 12, endianMode);
        int direction = Modbus_ReadInt32(&pucRegBuffer[0], 14, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[0], 16, endianMode);
        ret = adx_MoveRelative(axisId, index, pos, vel, acc, dec, jerk, direction, bufferMode);
        if (ret)
            ULOG_ERROR("adx_MoveRelative error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_MoveRelative finish\n");
        break;
    }

    //====================================================
    case cmd_adx_MoveVelocity_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_MoveVelocity\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        double vel = Modbus_ReadFloat(&pucRegBuffer[0], 4, endianMode);
        double acc = Modbus_ReadFloat(&pucRegBuffer[0], 6, endianMode);
        double dec = Modbus_ReadFloat(&pucRegBuffer[0], 8, endianMode);
        double jerk = Modbus_ReadFloat(&pucRegBuffer[0], 10, endianMode);
        int direction = Modbus_ReadInt32(&pucRegBuffer[0], 12, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[0], 14, endianMode);
        ret = adx_MoveVelocity(axisId, index, vel, acc, dec, jerk, direction, bufferMode);
        if (ret)
            ULOG_ERROR("adx_MoveVelocity error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_MoveVelocity finish\n");
        break;

    }
    //====================================================
    case cmd_adx_Halt_ADDR:
    {
        ULOG_INFO("cmdExecute: cmd_adx_Halt\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        double deceleration = Modbus_ReadFloat(&pucRegBuffer[0], 4, endianMode);
        double jerk = Modbus_ReadFloat(&pucRegBuffer[0], 6, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[0], 8, endianMode);
        ret = adx_Halt(axisId, index, deceleration, jerk, bufferMode);
        if (ret)
            ULOG_ERROR("adx_Halt error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_Halt finish\n");
        break;
    }
    //====================================================
    case cmd_adx_Stop:
    {
        ULOG_INFO("cmdExecute: cmd_adx_Stop\n");
        int axisId = Modbus_ReadInt32(&pucRegBuffer[0], 0, endianMode);
        int index = Modbus_ReadInt32(&pucRegBuffer[0], 2, endianMode);
        double deceleration = Modbus_ReadFloat(&pucRegBuffer[0], 4, endianMode);
        double jerk = Modbus_ReadFloat(&pucRegBuffer[0], 6, endianMode);
        int bufferMode = Modbus_ReadInt32(&pucRegBuffer[0], 8, endianMode);
        ret = adx_Stop(axisId, index, deceleration, jerk, bufferMode);
        if (ret)
            ULOG_ERROR("adx_Stop error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_Stop finish\n");
        break;
    }
    case cmd_adx_SetGantryConfig:
    {
        ULOG_INFO("cmdExecute: cmd_adx_SetGantryConfig\n");
        GantryConfig_t cfg;
        int gantryId=Modbus_ReadInt32(&pucRegBuffer[0],0,endianMode);
        cfg.masterId=Modbus_ReadInt32(&pucRegBuffer[2],0,endianMode);
        cfg.slaveId=Modbus_ReadInt32(&pucRegBuffer[4],0,endianMode);
        cfg.masterOffset_mm=Modbus_ReadFloat(&pucRegBuffer[6],0,endianMode);
        cfg.slaveOffset_mm=Modbus_ReadFloat(&pucRegBuffer[8],0,endianMode);
        cfg.offsetspeed_mm_s=Modbus_ReadFloat(&pucRegBuffer[10],0,endianMode);
        ret=adx_SetGantryConfig(gantryId,&cfg);
        if(ret)
            ULOG_ERROR("adx_SetGantryConfig error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_SetGantryConfig finish\n");
        break;
    }
    case cmd_adx_SetGantryEnable:
    {
        ULOG_INFO("cmdExecute: cmd_adx_SetGantryEnable\n");
        int gantryId=Modbus_ReadInt32(&pucRegBuffer[0],0,endianMode);
        int enable=Modbus_ReadInt32(&pucRegBuffer[2],0,endianMode);
        ret=adx_SetGantryEnable(gantryId,enable);
        if(ret)
            ULOG_ERROR("adx_SetGantryEnable error ret=%d", ret);
        ULOG_INFO("cmdExecute: cmd_adx_SetGantryEnable finish\n");
        break;
    }
  

    //====================================================
    case cmd_adx_WriteFloatParam:
    {


        break;
    }


    //====================================================
    default:
        ret = CMD_NOT_FOUND_ERR; // 未知命令
    }


	return 0;
}


int excuteReadCmd(unsigned char* pucRegBuffer, int usAddress, int mbMode)
{
  (void)mbMode;
    int ret = 0;
    ENDIAN_MODE endianMode = ENDIAN_BYTEWORD_SWAP;
    int address = usAddress - 1;
    //分地址段寻址
    if (address >= cmd_adx_ReadMultiParam_ADDR &&address < cmd_adx_GroupReadMultiParam_ADDR)
    {
        int foundCmd = 0;
        for (int i = 0; i < 4; i++)
        {
            if(address ==cmd_adx_ReadMultiParam_ADDR+i*AXIS_MULTIPARAM_ADDR_OFFSET)
              {
                foundCmd = 1;
                //读取轴参数结构体
                //根据地址计算轴ID
                int axisId = -1;
                int index = 0;
                for (int i = 0; i < 32; i++)
                {
                    if (usAddress - 1 == cmd_adx_ReadMultiParam_ADDR + i * 120)
                    {
                        axisId = i;
                        break;
                    }
                }
                ULOG_INFO("cmdExecute: cmd_adx_ReadMultiParam axisId=%d\n", axisId);

                axisParam_t param;
                ret = adx_ReadMultiParameters(axisId, index, &param);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadMultiParameters error ret=%d", ret);
					//错误码写入寄存器缓冲区
                    //待开发
					return ret;
                }

                //将读取到的结构体参数写入到寄存器缓冲区
                int idx = 0;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.ppr, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.dirPol, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.speed, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.acc, endianMode);
                idx = idx + 4;
                Modbus_WriteFloat(&pucRegBuffer[idx], 0, param.dpr, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.org, endianMode);
                idx = idx + 4;
                Modbus_WriteFloat(&pucRegBuffer[idx], 0, param.mdis, endianMode);
                idx = idx + 4;
                Modbus_WriteFloat(&pucRegBuffer[idx], 0, param.pdis, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.resetDir, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.resetSpeed, endianMode);
                break;
			}
        }
		if (!foundCmd)
		{
			ULOG_ERROR("cmdExecute: Command not found address=%d", usAddress);
			return CMD_NOT_FOUND_ERR;
		}
	}
    else if (address >= cmd_adx_GroupReadMultiParam_ADDR && address < cmd_adx_ReadTargetPos_ADDR)
    {
        //读轴组结构体参数
        int foundCmd = 0;
        for (int i = 0; i < 4; i++)
        {
            if (address == cmd_adx_GroupReadMultiParam_ADDR + i * GROUP_MULTIPARAM_ADDR_OFFSET)
            {
                foundCmd = 1;
                //读取轴组参数结构体
                //根据地址计算轴组ID
                int groupId = i;
                int index = 0;
                ULOG_INFO("cmdExecute: cmd_adx_GroupReadMultiParam groupId=%d\n", groupId);
                GrpParam_t param;
                int bufferMode = 0;
                ret = adx_GroupReadMultiParameters(groupId, index, &param, bufferMode);
                if (ret)
                {
                    ULOG_ERROR("adx_GroupReadMultiParameters error ret=%d", ret);
                    //错误码写入寄存器缓冲区
                    //待开发
                    return ret;
                }
                //将读取到的结构体参数写入到寄存器缓冲区
                int idx = 0;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.leftCmdBufferCount, endianMode);
                idx = idx + 4;
                Modbus_WriteInt32(&pucRegBuffer[idx], 0, param.totalCmdCount, endianMode);
                idx = idx + 4;
                Modbus_WriteFloat(&pucRegBuffer[idx], 0, param.curRunningCmdIndex, endianMode);
                idx = idx + 4;
                Modbus_WriteFloat(&pucRegBuffer[idx], 0, param.status, endianMode);
                break;
            }
        }
        if (!foundCmd)
        {
			ULOG_ERROR("cmdExecute: Command not found address=%d", usAddress);
			return CMD_NOT_FOUND_ERR;
        }
    }
    else if(address>=cmd_adx_ReadTargetPos_ADDR && address< cmd_adx_ReadTargetVel_ADDR)
    {
        int foundCmd = 0;
        for(int i=0;i<32;i++)
        {
            if(address>=cmd_adx_ReadTargetPos_ADDR+i*2)
            {
                foundCmd=1;
                //读取轴目标位置
                double targetPos=0.0;
                int axisId=i;
                ret=adx_ReadTargetPos(axisId,0,&targetPos);
                if(ret)
                {
                    ULOG_ERROR("adx_ReadTargetPos error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, targetPos, endianMode);
                break;
            }
        }
        if(!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }

    }
    else if(address>= cmd_adx_ReadLogicalPos_ADDR && address< cmd_adx_ReadLogicalVel_ADDR)
    {
        int foundCmd = 0;
        for(int i=0;i<32;i++)
        {
            if(address==cmd_adx_ReadLogicalPos_ADDR+i*2)
            {
                foundCmd=1;
                //读取轴逻辑位置
                double logicalPos=0.0;
                int axisId=i;
                ret=adx_ReadLogicalPos(axisId,0,&logicalPos);
                if(ret)
                {
                    ULOG_ERROR("adx_ReadLogicalPos error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, logicalPos, endianMode);
                break;
            }
        }
        if(!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }

    }
    else if(address>= cmd_adx_ReadLogicalVel_ADDR && address< cmd_adx_ReadLogicalAcc_ADDR)
    {
        int foundCmd = 0;
        for(int i=0;i<32;i++)
        {
            if(address==cmd_adx_ReadLogicalVel_ADDR+i*2)
            {
                foundCmd=1;
                //读取轴逻辑速度
                double logicalVel=0.0;
                int axisId=i;
                ret=adx_ReadLogicalVel(axisId,0,&logicalVel);
                if(ret)
                {
                    ULOG_ERROR("adx_ReadLogicalVel error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, logicalVel, endianMode);
                break;
            }
        }
        if(!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }

    }
    else if (address >= cmd_adx_ReadLogicalAcc_ADDR && address < cmd_adx_ReadLogicalJerk_ADDR)
	{
		int foundCmd = 0;
		for (int i = 0; i < 32; i++)
		{
			if (address == cmd_adx_ReadLogicalAcc_ADDR + i * 2)
			{
				foundCmd = 1;
				//读取轴逻辑加速度
				double logicalAcc = 0.0;
				int axisId = i;
				ret = adx_ReadLogicalAcc(axisId,0, &logicalAcc);
				if (ret)
				{
					ULOG_ERROR("adx_ReadLogicalAcc error ret=%d", ret);
					return ret;
				}
				Modbus_WriteFloat(&pucRegBuffer[0], 0, logicalAcc, endianMode);
				break;
			}
		}
		if (!foundCmd)
		{
			ULOG_ERROR("cmdExecute: Command not found address=%d", address);
			return CMD_NOT_FOUND_ERR;
		}
	}
    else if (address >= cmd_adx_ReadLogicalJerk_ADDR && address < cmd_adx_ReadActualPos_ADDR)
    {
		int foundCmd = 0;
        for (int i = 0; i < 32; i++)
        {
			if (address == cmd_adx_ReadLogicalJerk_ADDR + i * 2)
			{
				foundCmd = 1;
				//读取轴逻辑加加速度
				double logicalJerk = 0.0;
				int axisId = i;
				ret = adx_ReadLogicalJerk(axisId, 0,&logicalJerk);
				if (ret)
				{
					ULOG_ERROR("adx_ReadLogicalJerk error ret=%d", ret);
					return ret;
				}
				Modbus_WriteFloat(&pucRegBuffer[0], 0, logicalJerk, endianMode);
				break;
			}
        }
        if (!foundCmd)
        {
			ULOG_ERROR("cmdExecute: Command not found address=%d", address);
			return CMD_NOT_FOUND_ERR;
        }
    }

    else if (address >= cmd_adx_ReadActualPos_ADDR && address < cmd_adx_ReadActualVel_ADDR)
    {
		int foundCmd = 0;
        for (int i = 0; i < 32; i++)
        {
            if (address == cmd_adx_ReadActualPos_ADDR + i * 2)
            {
                foundCmd = 1;
                //读取轴实际位置
                double actualPos = 0.0;
                int axisId = i;
                ret = adx_ReadActualPos(axisId,0, &actualPos);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadActualPos error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, actualPos, endianMode);
                break;
            }

        }
        if (!foundCmd)
        {
			ULOG_ERROR("cmdExecute: Command not found address=%d", address);
			return CMD_NOT_FOUND_ERR;

        }
	}
    else if (address >= cmd_adx_ReadActualVel_ADDR && address < cmd_adx_ReadActualAcc_ADDR)
    {
		int foundCmd = 0;
        for (int i = 0; i < 32; i++)
        {
            if (address == cmd_adx_ReadActualVel_ADDR + i * 2)
            {
                foundCmd = 1;
                //读取轴实际速度
                double actualVel = 0.0;
                int axisId = i;
                ret = adx_ReadActualPos(axisId, 0,&actualVel);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadActualPos error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, actualVel, endianMode);
                break;
            }
        }
        if (!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }

    }
    else if (address >= cmd_adx_ReadActualAcc_ADDR && address < cmd_adx_ReadActualJerk_ADDR)
    {
		int foundCmd = 0;
        for (int i = 0; i < 32; i++)
        {
            if (address == cmd_adx_ReadActualAcc_ADDR + i * 2)
            {
                foundCmd = 1;
                //读取轴实际加速度
                double actualAcc = 0.0;
                int axisId = i;
                ret = adx_ReadActualAcc(axisId,0, &actualAcc);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadActualAcc error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteFloat(&pucRegBuffer[0], 0, actualAcc, endianMode);
                break;
            }
        }
        if (!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }
    }
	else if (address >= cmd_adx_ReadActualJerk_ADDR && address < cmd_adx_ReadStatus_ADDR)
	{
		int foundCmd = 0;
		for (int i = 0; i < 32; i++)
		{
			if (address == cmd_adx_ReadActualJerk_ADDR + i * 2)
			{
				foundCmd = 1;
				//读取轴实际加加速度
				double actualJerk = 0.0;
				int axisId = i;
				ret = adx_ReadActualJerk(axisId, 0,&actualJerk);
				if (ret)
				{
					ULOG_ERROR("adx_ReadActualJerk error ret=%d", ret);
					return ret;
				}
				Modbus_WriteFloat(&pucRegBuffer[0], 0, actualJerk, endianMode);
				break;
			}
		}
		if (!foundCmd)
		{
			ULOG_ERROR("cmdExecute: Command not found address=%d", address);
			return CMD_NOT_FOUND_ERR;
		}
	}
    else if (address >= cmd_adx_ReadStatus_ADDR && address < cmd_adx_GroupReadTargetPos_ADDR)
    {
        int foundCmd = 0;
        for (int i = 0; i < 32; i++)
        {
            if (address == cmd_adx_ReadStatus_ADDR + i * 2)
            {
                foundCmd = 1;
                //读取轴状态
                int status = 0;
                int axisId = i;
                ret = adx_ReadStatus(axisId,0, &status);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadStatus error ret=%d", ret);
                    return ret;
                }
                Modbus_WriteInt32(&pucRegBuffer[0], 0, status, endianMode);
                break;
            }
        }
        if (!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }
	}
	else if (address >= cmd_adx_GroupReadTargetPos_ADDR && address < cmd_adx_GroupReadLogicalPos_ADDR)
	{
		int foundCmd = 0;
        for (int i = 0; i < 4; i++)
        {
			if (address == cmd_adx_GroupReadTargetPos_ADDR + i * 2)
			{
				foundCmd = 1;
				//读取轴组目标位置
                double targetPos[6] = { 0.0 };
				int groupId = i;
				ret = adx_GroupReadTargetPos(groupId, 0,targetPos);
				if (ret)
				{
					ULOG_ERROR("adx_ReadGroupTargetPos error ret=%d", ret);
					return ret;
				}
				for (int i = 0; i < 6; i++)
				    Modbus_WriteFloat(&pucRegBuffer[4*i], 0, targetPos[i], endianMode);
				break;
			}
        }
        if (!foundCmd)
        {
            ULOG_ERROR("cmdExecute: Command not found address=%d", address);
            return CMD_NOT_FOUND_ERR;
        }
    }
    else if (address >= cmd_adx_GroupReadLogicalPos_ADDR && address < cmd_adx_GroupReadLogicalVel_ADDR)
    {
		int foundCmd = 0;
        for (int i = 0; i < 4; i++)
        {
            if (address == cmd_adx_GroupReadLogicalPos_ADDR + i * 2)
            {
                foundCmd = 1;
                //读取轴组逻辑位置
                double logicalPos[6] = { 0.0 };
                int groupId = i;
                ret = adx_GroupReadLogicalPos(groupId, 0,logicalPos);
                if (ret)
                {
                    ULOG_ERROR("adx_ReadGroupLogicalPos error ret=%d", ret);
                    return ret;
                }
                for (int i = 0; i < 6; i++)
                    Modbus_WriteFloat(&pucRegBuffer[4 * i], 0, logicalPos[i], endianMode);
                break;
            }
        }
		if (!foundCmd)
		{
			ULOG_ERROR("cmdExecute: Command not found address=%d", address);
			return CMD_NOT_FOUND_ERR;
		}

    }
    else if(address>= cmd_adx_ReadFloatParam_ADDR&& address< cmd_adx_ReadIntParam_ADDR)
    {
		//实时轴参数
        int foundAxis = 0;
        int axisId = -1;
        for (int i = 0; i < 32; i++)
        {

            if (address >= cmd_adx_ReadFloatParam_ADDR + i * AXIS_PARAM_ADDR_OFFSET)
            {
                foundAxis = 1;
                axisId = i;
                break;
            }
        }
        if (!foundAxis)
        {
            //找不到对应轴号
			ULOG_ERROR("cmdExecute: Axis not found for address=%d", usAddress);
			return AXIS_NOT_FOUND_ERR;

        }
		ULOG_INFO("cmdExecute: cmd_adx_ReadFloatParam axisId=%d\n", axisId);
        //偏移地址寻址
	

	}
	else if (address >= cmd_adx_GroupReadFloatParam_ADDR && address < cmd_adx_GroupReadIntParam_ADDR)
	{
		//读实时轴组浮点参数
        int foundGroup = 0;
        int groupId = -1;
        for (int i = 0; i < 4; i++)
        {

            if (address >= cmd_adx_GroupReadFloatParam_ADDR + i * GROUP_PARAM_ADDR_OFFSET)
            {
                foundGroup = 1;
                groupId = i;
                break;
            }
        }
        if (!foundGroup)
        {
            //找不到对应轴组号
			ULOG_ERROR("cmdExecute: Group not found for address=%d", address);
			return GROUP_NOT_FOUND_ERR;
        }
		ULOG_INFO("cmdExecute: cmd_adx_GroupReadFloatParam groupId=%d\n", groupId);
        int paramId=0;
        //轴组targetPos,LogicalPos,LogicalVel,LogicalAcc,LogicalJerk，分别6个坐标，4个轴组，偏移地址寻址计算
        //6个连续浮点参数，前4个参数的ID计算方式
        if(address<=cmd_adx_GroupReadFloatParam_ADDR+groupId * GROUP_PARAM_ADDR_OFFSET+6*4*4)
        {
              paramId =+(address - (cmd_adx_GroupReadFloatParam_ADDR + groupId * GROUP_PARAM_ADDR_OFFSET)) / 24;
            
        }
        else 
        {   //其余单个浮点参数，偏移地址寻址计算
             paramId =  + (address - (cmd_adx_GroupReadFloatParam_ADDR + groupId * GROUP_PARAM_ADDR_OFFSET)) / 4;
        }
        (void)paramId;
    }
     
	return 0;
}