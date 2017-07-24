#include "Task.h"

/**
 * @Description IMU任务 500HZ
 */
OS_SEM IMU_proc;
void IMU_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	OSSemCreate ((OS_SEM*)&IMU_proc,
								(CPU_CHAR*)"IMU_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	float GlobalEuler[3];
	while(1)
	{
		OSSemPend (&IMU_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		//获取IMU解算后角度
		IMU_getInfo(GlobalEuler);
	}
}
