/****************************************************************************
 * examples/leds/leds_main.c
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

#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_led_daemon_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_daemon
 ****************************************************************************/

static int led_daemon(int argc, char *argv[])
{
  int cnt;
  int led_num;
  int led_flag;

  cnt = 0;
  led_num   = 0;
  led_flag  = 0;

  boardctl(RECORD_LED_GPIOINIT, 0);
  boardctl(RECORD_RELAY_GPIOINIT, 0);
  
  for (; ; )
    {
		do
		{
			//led_num = cnt++&3;
			led_num = cnt++&7;
		
			if(led_flag == 0)
			{
				switch(led_num)
				{
					case 0:
							boardctl(RECORD_WAKEUP_RESET_ENABLE, 0);
						break;
					case 1:
							boardctl(RECORD_BLUE_RESET_ENABLE, 0);
						break;
					case 2:
							boardctl(RECORD_GPS_RESET_ENABLE, 0);
						break;
					case 3:
							boardctl(RECORD_433M_RESET_ENABLE, 0);
						break;
					case 4:
							boardctl(RECORD_RELAY_RESET_ENABLE, 0);
						break;
				}
			}
			else
			{
				switch(led_num)
				{
					case 0:
							boardctl(RECORD_WAKEUP_RESET_DISABLE, 0);
						break;
					case 1:
							boardctl(RECORD_BLUE_RESET_DISABLE, 0);
						break;
					case 2:
							boardctl(RECORD_GPS_RESET_DISABLE, 0);
						break;
					case 3:
							boardctl(RECORD_433M_RESET_DISABLE, 0);
						break;
					case 4:
							boardctl(RECORD_RELAY_RESET_DISABLE, 0);
						break;
				}
			}

			
			//if(cnt %4 == 0)
			if(cnt %8 == 0)
			{
				led_flag = ~led_flag;
			}
			usleep(500*1000L);

		}
		while(1);
	    
    }

  printf("led_daemon: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * leds_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int boardctl_myleds_main(int argc, FAR char *argv[])
#endif
{
  int ret;

  printf("leds_main: Starting the led_daemon\n");
  if (g_led_daemon_started)
    {
      printf("leds_main: led_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("led_daemon", CONFIG_EXAMPLES_BOARDCTL_MYLEDS_PRIORITY,
                    CONFIG_EXAMPLES_BOARDCTL_MYLEDS_STACKSIZE, led_daemon,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("leds_main: ERROR: Failed to start led_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("leds_main: led_daemon started\n");
  while(1)
  {
  		sleep(1);
		printf("boardctl_myleds: running\n");
  }
  return EXIT_SUCCESS;
}
