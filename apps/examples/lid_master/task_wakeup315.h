#ifndef _TASK_WAKEUP315_H
#define _TASK_WAKEUP315_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "task_485.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private function
 ****************************************************************************/

void	Wakeup_openLock(struct msg485 *msg485);
int 	wakeup_315(int argc, char *argv[]);



#ifdef __cplusplus
}
#endif

#endif 
