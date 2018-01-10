/****************************************************************************
 * examples/bluetooth/bluetooth_main.c
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

#include "task_bluetooth.h"
#include "task_gprs.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool  g_bluetooth_started;

struct msgbt bluetooth_msg;
int  fd_bluetooth;


/****************************************************************************
 * Name: bluetooth_init
 ****************************************************************************/
int bluetooth_init(void)
{
	int fd_blue;
	fd_blue = open(CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_blue < 0)
	{
		int errcode = errno;
		printf("bluetooth: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, errcode);
	}

	printf("boardctl init bluetooth gpiopin !\n");

	//gpio init
	boardctl(BOARDIOC_BLUEDEV_GPIOINIT, 0);
    //power on
	boardctl(BOARDIOC_BLUEDEV_POWER_ENABLE, 0);			//power on
	usleep(100*1000L);
	//gpio  hight------ready
	boardctl(BOARDIOC_BLUEDEV_WAKEUP_DISABLE, 0);			//gpio  hight------ready
	usleep(100*1000L);
	//wakeup	
	boardctl(BOARDIOC_BLUEDEV_WAKEUP_ENABLE, 0);			//gpio  lower
	usleep(1200*1000L);
	//go data mode 
	boardctl(BOARDIOC_BLUEDEV_WAKEUP_DISABLE, 0);			//gpio  hight

	//open msg notifier
	write(fd_blue,"AT+NOTI[Y]",strlen("AT+NOTI[Y]"));
	usleep(1000*1000L);                                     
	//set name 
	write(fd_blue,"AT+NAME[BLUE-TEST]",strlen("AT+NAME[BLUE-TEST]"));
	usleep(1000*1000L);                                     
	//set mode slave 
	write(fd_blue,"AT+ROLE[P]",strlen("AT+ROLE[P]"));
	usleep(1000*1000L);
	//set transmits 4db 
	write(fd_blue,"AT+POWE[D]",strlen("AT+POWE[D]"));

   close(fd_blue);

   return 0;
}


/****************************************************************************
 * Name: bluetooth
 ****************************************************************************/

int master_bluetooth(int argc, char *argv[])
{
	struct timeval timeout;
	char 	cArray[64];
	fd_set 	rfds;	
	int  	iRet = 0;
	int  	iBytes = 0;

	g_bluetooth_started = true;
	fd_bluetooth = open(CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_bluetooth < 0)
	{
		int errcode = errno;
		printf("bluetooth: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, errcode);
		goto errout;
	}
	bluetooth_init();	
	boardctl(BOARDIOC_BLUEDEV_POWER_DISABLE, 0);			//power off
	while(1)
	{
	
		FD_ZERO(&rfds);											
		FD_SET(fd_bluetooth, &rfds);									
		timeout.tv_sec = 120;
		timeout.tv_usec = 0;
		iRet = select(fd_bluetooth+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			//printf("bluetooth_dev rcv timeout!!!\n");
			boardctl(BOARDIOC_BLUEDEV_POWER_DISABLE, 0);			//power off
		}
		else
		{
			if(FD_ISSET(fd_bluetooth, &rfds)) 
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(cArray, '\0', sizeof(cArray));
				iBytes = read(fd_bluetooth, cArray, sizeof(cArray));
			    tcflush(fd_bluetooth, TCIFLUSH);

				printf("Read <%d>Bytes Data:%s\n",iBytes,cArray);
				if(strncmp("$BT_OPEN_LOCK,ASK,EOF",cArray,strlen("$BT_OPEN_LOCK,ASK,EOF")) == 0)
				{
					bluetooth_msg.type	= BT_OPEN_LOCK;
					GprsData.process_state = FAIL;
					printf("rcv bluetooth open lock\n");
					//ack
					memset(cArray, '\0', sizeof(cArray));
					sprintf(cArray,"GPRS:get the lock configuration form server......\n");
				    iBytes = write(fd_bluetooth,cArray,strlen(cArray));
					if(iBytes == -1)
					{
						printf("Error:write  Data to Bluetooth\n");
					}
				}
				
			}
		}
	}
	    
errout:
  g_bluetooth_started = false;

  printf("bluetooth: Terminating\n");
  return EXIT_FAILURE;
}

