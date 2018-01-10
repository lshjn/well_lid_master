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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool  g_485_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static	int  fd_485;

/****************************************************************************
 * Name: 485
 ****************************************************************************/
/*
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
	
	tcflush(fd_485, TCIFLUSH);
   	if ((tcsetattr(fd_485, TCSANOW, &newtio)) != 0) {  	// activate settings
              perror("set device param error\n");
	      exit(1);		
	}
		
	return 0;
} 	
*/

static int slave485(int argc, char *argv[])
{
	struct timeval timeout;
	char 	cArray[64];
	fd_set 	rfds;	
	int  	iRet = 0;
	int  	iBytes = 0;
    int     cnt = 0;
	
	g_485_started = true;

	fd_485 = open(CONFIG_EXAMPLES_485_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_485 < 0)
	{
		int errcode = errno;
		printf("485: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_485_DEVPATH, errcode);
		goto errout;
	}


	//Uart_init();
		
	while(1)
	{
	
		FD_ZERO(&rfds);											
		FD_SET(fd_485, &rfds);									
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		iRet = select(fd_485+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			printf("485_dev rcv timeout!!!\n");
			
		    iBytes = write(fd_485,"this is test txt 0123456789\n",strlen("this is test txt 0123456789\n"));
			if(iBytes > 0)
			{
				printf("write <%d>Bytes Data to 485\n",iBytes);
			}
			else if(iBytes == -1)
			{
				printf("Error:write  Data to 485\n");
			}
		}
		else
		{
			if(FD_ISSET(fd_485, &rfds)) 
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(cArray, '\0', sizeof(cArray));
				iBytes = read(fd_485, cArray, sizeof(cArray));
			    tcflush(fd_485, TCIFLUSH);

				//if(strncmp("test",cArray,4) != 0)
				{
					printf("test <%d>Read <%d>Bytes Data:%s\n",cnt++,iBytes,cArray);
				}
				/*
				//»ØÐ´Êý¾Ý
			    iBytes = write(fd_485,cArray,iBytes);
				if(iBytes > 0)
				{
					if(strncmp("test",cArray,4) != 0)
					{
						printf("test write <%d>Bytes Data:%s\n",iBytes,cArray);
					}
				}
				else if(iBytes == -1)
				{
					printf("Error:write  Data to 485\n");
				}
				*/
			}
		}
	}
	    
errout:
  g_485_started = false;

  printf("485: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * slave485_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int slave485_main(int argc, FAR char *argv[])
#endif
{
  int ret;

  printf("slave485_main: Starting the slave485_main\n");
  if (g_485_started)
    {
      printf("slave485_main: slave485_main already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("slave485", CONFIG_EXAMPLES_485_PRIORITY,
                    CONFIG_EXAMPLES_485_STACKSIZE, slave485,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("slave485_main: ERROR: Failed to start 485: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("slave485_main: 485 started\n");
  while(1)
  {
  		sleep(1);
		//printf("slave485_main: running\n");
  }
  return EXIT_SUCCESS;
}
