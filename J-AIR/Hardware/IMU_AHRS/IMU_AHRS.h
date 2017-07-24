#ifndef __IMU_AHRS_H
#define __IMU_AHRS_H

extern volatile float RDrone_R[3][3];


void IMU_HardwareInit(void);
void IMU_getInfo(float * angles);

#endif


