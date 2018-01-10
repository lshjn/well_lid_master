/****************************************************************************
 * examples/485.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <signal.h>
#include <pthread.h>

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include "task_adc.h"
#include "task_485.h"
#include "task_gprs.h"
#include "task_bluetooth.h"
#include "task_motor.h"
#include "task_monitor.h"
#include "task_wakeup315.h"
#include "task_close315.h"
#include "task_zd801s.h"
#include "task_flash.h"



/****************************************************************************
 * lid_master_main
 * liushuhe
 * 2017.09.26
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int lid_master_main(int argc, FAR char *argv[])
#endif
{

  int ret;

  ret = task_create("master_monitor", CONFIG_EXAMPLES_MONITOR_PRIORITY,
                    CONFIG_EXAMPLES_MONITOR_STACKSIZE, master_monitor,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_monitor: ERROR: Failed to start monitor: %d\n",
             errcode);
      return EXIT_FAILURE;
    }
  ret = task_create("master_485", CONFIG_EXAMPLES_485_PRIORITY,
                    CONFIG_EXAMPLES_485_STACKSIZE, master_485,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_485: ERROR: Failed to start 485: %d\n",errcode);
      return EXIT_FAILURE;
    }

  ret = task_create("gprs", CONFIG_EXAMPLES_GPRS_PRIORITY,
                    CONFIG_EXAMPLES_GPRS_STACKSIZE, master_gprs,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("gprs_main: ERROR: Failed to start gprs: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  ret = task_create("bluetooth", CONFIG_EXAMPLES_BLUETOOTH_PRIORITY,
                    CONFIG_EXAMPLES_BLUETOOTH_STACKSIZE, master_bluetooth,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bluetooth_main: ERROR: Failed to start bluetooth: %d\n",
             errcode);
      return EXIT_FAILURE;
    }
  

  ret = task_create("master_motor", CONFIG_EXAMPLES_MOTOR_PRIORITY,
                    CONFIG_EXAMPLES_MOTOR_STACKSIZE, master_motor,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_motor: ERROR: Failed to start motor: %d\n",errcode);
      return EXIT_FAILURE;
    }
  
  ret = task_create("master_wakeup_315", CONFIG_EXAMPLES_WKUP315_PRIORITY,
                    CONFIG_EXAMPLES_WKUP315_STACKSIZE, wakeup_315,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_wakeup: ERROR: Failed to start wakeup_315: %d\n",
             errcode);
      return EXIT_FAILURE;
    }
  ret = task_create("master_close_315", CONFIG_EXAMPLES_WKUP315_PRIORITY,
                    CONFIG_EXAMPLES_WKUP315_STACKSIZE,closelock_315,NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_close_315: ERROR: Failed to start closelock_315: %d\n",
             errcode);
      return EXIT_FAILURE;
    }


  ret = task_create("master_flash", CONFIG_EXAMPLES_FLASH_PRIORITY,
                    CONFIG_EXAMPLES_FLASH_STACKSIZE, master_flash,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("flash: ERROR: Failed to start flash: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  ret = task_create("master_zd801s", CONFIG_EXAMPLES_ZD801S_PRIORITY,
                    CONFIG_EXAMPLES_ZD801S_STACKSIZE, alarm_zd801s,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("flash: ERROR: Failed to start flash: %d\n",
             errcode);
      return EXIT_FAILURE;
    }
  
  ret = task_create("master_adc", CONFIG_EXAMPLES_ADC_PRIORITY,
                    CONFIG_EXAMPLES_ADC_STACKSIZE, master_adc,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_adc: ERROR: Failed to start adc: %d\n",
             errcode);
      return EXIT_FAILURE;
    }


  while(1)	
  {
  		usleep(500*1000);
		//*((int *)(0x00100000)) = 0;
		getSystime_check();
		#if 1//for to test
  		if((1==CheckTimeInt())&&(0 == TimeInt_SampleFlag))
  		{
  			pthread_mutex_lock(&g_TimIntMutex);
			TimeInt_SampleFlag = 1;
			printf("timeint......\n");
			pthread_mutex_unlock(&g_TimIntMutex);
		}
		#endif
		//boardctl(BOARDIOC_LED_WAKEUP_ENABLE, 0);
		//boardctl(BOARDIOC_LED_DEBUG_ENABLE, 0);		
        
		//sleep(1);
		//boardctl(BOARDIOC_LED_WAKEUP_DISABLE, 0);
		//boardctl(BOARDIOC_LED_DEBUG_DISABLE, 0);

		
		
  }

  return EXIT_SUCCESS;
}
