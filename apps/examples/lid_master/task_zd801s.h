#ifndef _TASK_ZD801S_H
#define _TASK_ZD801S_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "task_485.h"
#include "task_motor.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private function
 ****************************************************************************/
void	zd801s_alarm(struct msg485 *msg485,struct hall_sensor *Sensor);
int 	wakeup_zd801s(int argc, char *argv[]);
int 	alarm_zd801s(int argc, char *argv[]);



#ifdef __cplusplus
}
#endif

#endif 
