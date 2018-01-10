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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool  g_bluetooth_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static	int  fd_blue;

/****************************************************************************
 * Name: bluetooth
 ****************************************************************************/
static int Uart_init(void)
{	
	struct termios newtio;

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD; 
	newtio.c_cflag |= CS8;	      						// 8 bit
	newtio.c_cflag &= ~CSTOPB;    						// 1 stop bit
	newtio.c_cflag &= ~PARENB;    						// none parity
	cfsetispeed(&newtio, B9600);

	printf("Uart4 -> 9600B, N, 8, 1 \n");
	
	tcflush(fd_blue, TCIFLUSH);
   	if ((tcsetattr(fd_blue, TCSANOW, &newtio)) != 0) {  	// activate settings
              perror("set device param error\n");
	      exit(1);		
	}
		
	return 0;
} 	

static int bluetooth(int argc, char *argv[])
{
	struct timeval timeout;
	char 	cArray[64];
	fd_set 	rfds;	
	int  	iRet = 0;
	int  	iBytes = 0;

	printf("111111111 !\n");
	g_bluetooth_started = true;

	fd_blue = open(CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_blue < 0)
	{
		int errcode = errno;
		printf("bluetooth: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_BLUETOOTH_DEVPATH, errcode);
		goto errout;
	}

	printf("boardctl init bluetooth gpiopin !\n");
	
	boardctl(RECORD_BLUEDEV_GPIOINIT, 0);

	//wakeup	
	boardctl(RECORD_BLUEDEV_WAKEUP_ENABLE, 0);			//gpio  lower
	usleep(1200*1000L);
	boardctl(RECORD_BLUEDEV_WAKEUP_DISABLE, 0);			//gpio  hight

	
	write(fd_blue,"AT+MODE2",strlen("AT+MODE2"));
	usleep(1000*1000L);                                     //sleep 100ms
	write(fd_blue,"AT+UART0",strlen("AT+UART0"));
	usleep(1000*1000L);                                     //sleep 100ms
	write(fd_blue,"AT+NAMEBLUE-TEST1",strlen("AT+NAMEBLUE-TEST1"));

	//Uart_init();
		
	while(1)
	{
	
		FD_ZERO(&rfds);											
		FD_SET(fd_blue, &rfds);									
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		iRet = select(fd_blue+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			printf("bluetooth_dev rcv timeout!!!\n");
			
			//iBytes = write(fd_blue,"AT+VERS?",strlen("AT+VERS?"));

		    iBytes = write(fd_blue,"this is test txt 01\n",strlen("this is test txt 01\n"));
		    //iBytes = write(fd_blue,"this is test txt\n",strlen("this is test txt\n"));
			if(iBytes > 0)
			{
				printf("write <%d>Bytes Data to Bluetooth\n",iBytes);
			}
			else if(iBytes == -1)
			{
				printf("Error:write  Data to Bluetooth\n");
			}
		}
		else
		{
			if(FD_ISSET(fd_blue, &rfds)) 
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(cArray, '\0', sizeof(cArray));
				iBytes = read(fd_blue, cArray, sizeof(cArray));
			    tcflush(fd_blue, TCIFLUSH);

				printf("Read <%d>Bytes Data:%s\n",iBytes,cArray);
				
				//»ØÐ´Êý¾Ý
			    iBytes = write(fd_blue,cArray,iBytes);
				if(iBytes > 0)
				{
					printf("write <%d>Bytes Data:%s\n",iBytes,cArray);
				}
				else if(iBytes == -1)
				{
					printf("Error:write  Data to Bluetooth\n");
				}
			}
		}
	}
	    
errout:
  g_bluetooth_started = false;

  printf("bluetooth: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bluetooth_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int bluetooth_main(int argc, FAR char *argv[])
#endif
{
  int ret;
  printf("bluetooth_main: Starting the bluetooth\n");
  if (g_bluetooth_started)
    {
      printf("bluetooth_main: bluetooth already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("bluetooth", CONFIG_EXAMPLES_BLUETOOTH_PRIORITY,
                    CONFIG_EXAMPLES_BLUETOOTH_STACKSIZE, bluetooth,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("bluetooth_main: ERROR: Failed to start bluetooth: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("bluetooth_main: bluetooth started\n");
  while(1)
  {
  		sleep(1);
		//printf("bluetooth_main: running\n");
  }
  return EXIT_SUCCESS;
}
