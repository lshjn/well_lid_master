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
  userled_set_t supported;
  userled_set_t ledset;
  bool incrementing;
  int ret;
  int fd;
  struct userled_s  myleds;
  int led_num;
  int led_flag;

  printf("led_daemon: Running\n");


  printf("led_daemon: Opening %s\n", CONFIG_EXAMPLES_LEDS_DEVPATH);
  fd = open(CONFIG_EXAMPLES_LEDS_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      int errcode = errno;
      printf("led_daemon: ERROR: Failed to open %s: %d\n",
             CONFIG_EXAMPLES_LEDS_DEVPATH, errcode);
      goto errout;
    }


  ret = ioctl(fd, ULEDIOC_SUPPORTED,(unsigned long)((uintptr_t)&supported));

  if (ret < 0)
    {
      int errcode = errno;
      printf("led_daemon: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",
             errcode);
      goto errout_with_fd;
    }

  printf("led_daemon: Supported LEDs 0x%02x\n", (unsigned int)supported);
  supported &= CONFIG_EXAMPLES_LEDS_LEDSET;


  ledset       = 0;
  led_num      = 0;
  led_flag     = 0;
  for (; ; )
    {
		
		do
		{
			 
			myleds.ul_led = led_num++&3;
			if(led_flag == 0)
			{
				myleds.ul_on  = 0;
			}
			else
			{
				myleds.ul_on  = 1;
			}
			ret = ioctl(fd, ULEDIOC_SETLED, (unsigned long)&myleds);
			if (ret < 0)
			{
				int errcode = errno;
				printf("led_daemon: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",errcode);
				goto errout_with_fd;
			}

			if(led_num %4 == 0)
			{
				led_flag = ~led_flag;
			}
			usleep(500*1000L);

		}
		while(1);

		/*

		do
		{
			switch(led_num++%4)
			{
				case 0:
					ledset = 1;
					break;
				case 1:
					ledset = 2;
					break;
				case 2:
					ledset = 4;
					break;
				case 3:
					ledset = 8;
					break;
			}
			ret = ioctl(fd, ULEDIOC_SETALL, (unsigned long)ledset);
			if (ret < 0)
			{
				int errcode = errno;
				printf("led_daemon: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",
				     errcode);
				goto errout_with_fd;
			}

			usleep(500*1000L);

		}
		while(1);
	   */

	
    }

errout_with_fd:
  (void)close(fd);

errout:
  g_led_daemon_started = false;

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
int myleds_main(int argc, FAR char *argv[])
#endif
{
  int ret;

  printf("leds_main: Starting the led_daemon\n");
  if (g_led_daemon_started)
    {
      printf("leds_main: led_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("led_daemon", CONFIG_EXAMPLES_LEDS_PRIORITY,
                    CONFIG_EXAMPLES_LEDS_STACKSIZE, led_daemon,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("leds_main: ERROR: Failed to start led_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("leds_main: led_daemon started\n");
  return EXIT_SUCCESS;
}
