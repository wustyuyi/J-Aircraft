#include "stm32f4xx.h"
#include "explore_system.h"
#include "explore_systick.h"
#include "explore_usart.h"
#include "includes.h"
#include "os_app_hooks.h"
 
#define START_TASK_PRIO 3						                     // 任务优先级
#define START_STK_SIZE 512						                   // 任务堆栈大小
OS_TCB StartTaskTCB;							                       // 任务控制块
CPU_STK START_TASK_STK[START_STK_SIZE];					         // 任务堆栈
void start_task(void *p_arg);						                 // 任务函数


#define FLOAT_TASK_PRIO 6						                     // 任务优先级
#define FLOAT_STK_SIZE 128					                     // 任务堆栈大小
OS_TCB	FloatTaskTCB;							                       // 任务控制块
__align(8) CPU_STK FLOAT_TASK_STK[FLOAT_STK_SIZE];			 // 任务堆栈
void float_task(void *p_arg);						                 // 任务函数

/**
 * @Description 主函数启动操作系统
 */
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	Systick_Init(168);																			// 时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					// 中断分组配置
	Usart_Init(115200);																			// 串口初始化
	OSInit(&err);																						// 初始化UCOSIII
	OS_CRITICAL_ENTER();																		// 进入临界区
	OSTaskCreate(																						// 创建开始任务
		(OS_TCB*)&StartTaskTCB,																// 任务控制块
		(CPU_CHAR*)"start task", 															// 任务名字
		(OS_TASK_PTR)start_task, 															// 任务函数
		(void*)0,																							// 传递给任务函数的参数
		(OS_PRIO)START_TASK_PRIO,															// 任务优先级
		(CPU_STK*)&START_TASK_STK[0],													// 任务堆栈基地址
		(CPU_STK_SIZE)START_STK_SIZE/10,											// 任务堆栈深度限位
		(CPU_STK_SIZE)START_STK_SIZE,													// 任务堆栈大小
		(OS_MSG_QTY)0,																				// 任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
		(OS_TICK)0,																						// 当使能时间片轮转时的时间片长度，为0时为默认长度，
		(void*)0,																							// 用户补充的存储区
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,			// 任务选项
		(OS_ERR*)&err																					// 存放该函数错误时的返回值
		);
	OS_CRITICAL_EXIT();																			// 退出临界区
	OSStart(&err);																					// 开启UCOSIII
	while(1);
}

/**
 * @Description 开始任务函数
 */
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);											// 统计任务
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN											// 如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();	
#endif

#if OS_CFG_SCHED_ROUND_ROBIN_EN											// 当使用时间片轮转的时候
	// 使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

	OS_CRITICAL_ENTER();															// 进入临界区
	OSTaskCreate(																			// 创建浮点测试任务
		(OS_TCB*)&FloatTaskTCB,
		(CPU_CHAR*)"float test task",
		(OS_TASK_PTR)float_task,
		(void*)0,
		(OS_PRIO)FLOAT_TASK_PRIO,
		(CPU_STK*)&FLOAT_TASK_STK[0],
		(CPU_STK_SIZE)FLOAT_STK_SIZE/10,
		(CPU_STK_SIZE)FLOAT_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err);
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);			// 挂起开始任务
	OS_CRITICAL_EXIT();																// 进入临界区
}


/**
 * @Description 浮点测试任务
 */
void float_task(void *p_arg)
{
	CPU_SR_ALLOC();
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_CRITICAL_ENTER();					// 进入临界区
		printf("float_num's value is : %.4f\r\n",float_num);
		OS_CRITICAL_EXIT();					// 退出临界区
		delay_ms(500);						// 延时500ms
	}
}

