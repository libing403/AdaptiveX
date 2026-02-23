#ifndef TESTFUN_H_
#define TESTFUN_H_
#ifdef __cplusplus
extern "C" {
#endif  

#define CMD_MAX_LEN 128
typedef int (*cmd_func_t)(void);

typedef struct
{
    const char *cmd;
    cmd_func_t  func;
    const char *help;
} cmd_entry_t;

extern cmd_entry_t cmd_table[];
extern int maxCmdCount;
int show_help(void);
int test_adx_ConnetModbus(void);

int test_adx_CoreInit();
int test_adx_CoreClose();
int test_adx_Power();
int test_adx_ReadMultiParameter();
int test_adx_WriteMultiParameter();
int test_adx_MoveAbsolute();
int test_adx_ReadTargetPos();
int test_adx_ReadTargetVel();
int test_adx_ReadLogicalPos();
int test_adx_ReadActualPos();
int test_adx_SetMotorScale();
int test_adx_SetGantryEnable();
int test_adx_SetGantryConfig();
int test_adx_GantryMove();

#ifdef __cplusplus
}
#endif

#endif // !TESTFUN_H_
 

