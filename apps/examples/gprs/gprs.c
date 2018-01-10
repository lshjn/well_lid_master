/****************************************************************************
 * examples/gprs.c
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

static bool  g_gprs_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static	int  fd_gprs;
static	int  fd_stdin;
static	int  fd_max;

struct  gprs_init
{
	char  InitOK;
	char  State;
	char  ATcmd_num;
};
/****************************************************************************
 * Name: gprs
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
	
	tcflush(fd_gprs, TCIFLUSH);
   	if ((tcsetattr(fd_gprs, TCSANOW, &newtio)) != 0) {  	// activate settings
              perror("set device param error\n");
	      exit(1);		
	}
		
	return 0;
} 	
*/
static int gprs(int argc, char *argv[])
{
	struct timeval timeout;
	char 	cArray[64];
	char 	cArray2[200];
	char 	cArraySend[64];
	
	fd_set 	rfds;	
	int  	iRet = 0;
	int  	iBytes = 0;
    int     cnt = 0;
    int     gpsup_flag = 0;

	
	struct  gprs_init  InitGprs;

	InitGprs.State		= 0;
	InitGprs.ATcmd_num	= 0;
	
	g_gprs_started = true;


	fd_gprs = open(CONFIG_EXAMPLES_GPRS_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_gprs < 0)
	{
		int errcode = errno;
		printf("gprs: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_GPRS_DEVPATH, errcode);
		goto errout;
	}
	fd_stdin = open("/dev/ttyS3",O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd_stdin < 0)
	{
		int errcode = errno;
		printf("gprs: ERROR: Failed to open %s: %d\n","/dev/ttyS3", errcode);
		goto errout;
	}


	if(fd_gprs > fd_stdin)
	{
		fd_max = fd_gprs;	
	}
	else
	{
		fd_max = fd_stdin;	
	}


	/*
	bluetooth
	*/
	boardctl(RECORD_BLUEDEV_GPIOINIT, 0);
	//wakeup	
	boardctl(RECORD_BLUEDEV_WAKEUP_ENABLE, 0);			//gpio  lower
	usleep(1200*1000L);
	boardctl(RECORD_BLUEDEV_WAKEUP_DISABLE, 0);			//gpio  hight
	///////////////////////////////////////////////////////////////////////
	//gprs
	boardctl(GPRS_PWRON, 0);
	usleep(200*1000);
	boardctl(GPRS_WAKEUP, 0);
	usleep(200*1000);
	//boardctl(GPRS_RST, 0);
	
    iBytes = write(fd_gprs,"AT^SSYNC=1\r\n",strlen("AT^SSYNC=1\r\n"));									//开启led状态灯
	usleep(2000*1000);
	while(1)
	{
	
		FD_ZERO(&rfds);											
		FD_SET(fd_gprs, &rfds);									
		FD_SET(fd_stdin, &rfds);									
		timeout.tv_sec = 30;
		timeout.tv_usec = 0;
		iRet = select(fd_max+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			printf("gprs_dev rcv timeout!!!\n");
							//初始化
				//if(strcmp(cArray2,"init") == 0)
				{
					InitGprs.ATcmd_num = 1;
					InitGprs.State     = 1;
					write(fd_gprs,"AT\r\n",strlen("AT\r\n"));		//握手
				}

		}
		else
		{
			if(FD_ISSET(fd_gprs, &rfds)) 
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(cArray, '\0', sizeof(cArray));
				iBytes = read(fd_gprs, cArray, sizeof(cArray));
			    tcflush(fd_gprs, TCIFLUSH);

				printf("fd_gprs <%d>Read <%d>Bytes Data:%s\n",cnt++,iBytes,cArray);	
 				/************************************************************************************/
				if(strstr(cArray,"OK") != NULL)
				{
					InitGprs.State     = 1;
					printf("GPRS Return\"OK\" \n");
				}
 				/************************************************************************************/
				//注册gprs,建立tcp链接
				if(InitGprs.State == 1)
				{
					switch(InitGprs.ATcmd_num)
					{
						case 1:
							iBytes = write(fd_gprs,"AT^SICS=0,conType,GPRS0\r\n",strlen("AT^SICS=0,conType,GPRS0\r\n"));		//选择链接方式GPRS/CSD
							InitGprs.ATcmd_num = 2;
							InitGprs.State     = 0;
							break;
						case 2:
							iBytes = write(fd_gprs,"AT^SICS=0,apn,cment\r\n",strlen("AT^SICS=0,apn,cment\r\n"));				//设置APN
							InitGprs.ATcmd_num = 3;
							InitGprs.State     = 0;
							break;
						case 3:
							iBytes = write(fd_gprs,"AT^SISS=0,srvType,Socket\r\n",strlen("AT^SISS=0,srvType,Socket\r\n"));		//第0个服务平台 类型socket
							InitGprs.ATcmd_num = 4;
							InitGprs.State     = 0;
							break;
						case 4:
							iBytes = write(fd_gprs,"AT^SISS=0,conId,0\r\n",strlen("AT^SISS=0,conId,0\r\n"));					//指定internet链接平台，sics所设置的
							InitGprs.ATcmd_num = 5;
							InitGprs.State     = 0;
							break;
						case 5:
							iBytes = write(fd_gprs,"AT^SISS=0,address,socktcp://47.95.250.14:443\r\n",strlen("AT^SISS=0,address,socktcp://47.95.250.14:443\r\n"));		//设置ip地址及端口
							InitGprs.ATcmd_num = 6;
							InitGprs.State     = 0;
							break;
						case 6:
							iBytes = write(fd_gprs,"AT^SISO=0\r\n",strlen("AT^SISO=0\r\n"));									//opens the Socket service.
							InitGprs.ATcmd_num = 0;
							InitGprs.State     = 0;
							InitGprs.InitOK    = 1;
							break;
	 				}
				}

				if(InitGprs.ATcmd_num != 0)
				{
					continue;
				}
				else if(InitGprs.InitOK == 1)
				{
					printf("init ok\n");
				}
 				/************************************************************************************/
			}
			else if(FD_ISSET(fd_stdin, &rfds))
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(cArray2, '\0', sizeof(cArray2));
				iBytes = read(fd_stdin, cArray2, sizeof(cArray2));
			    tcflush(fd_stdin, TCIFLUSH);

				printf("fd_stdin <%d>Read <%d>Bytes Data:%s\n",cnt++,iBytes,cArray2);
				/*****************************************************************************************88*/
				//初始化
				if(strcmp(cArray2,"init") == 0)
				{
					InitGprs.ATcmd_num = 1;
					InitGprs.State     = 1;
					write(fd_gprs,"AT\r\n",strlen("AT\r\n"));		//握手
				}
				/*****************************************************************************************88*/
				//写数据
				if(strcmp(cArray2,"w") == 0)
				{
					iBytes = write(fd_gprs,"AT^SISW=0,100\r\n",strlen("AT^SISW=0,100\r\n"));										//Enter Transparent Access Mode
					usleep(200*1000);
					gpsup_flag = 2;
				}
				//读数据
				else if(strcmp(cArray2,"r") == 0)
				{
					iBytes = write(fd_gprs,"AT^SISR=0,100\r\n",strlen("AT^SISR=0,100\r\n"));										//Enter Transparent Access Mode
					usleep(200*1000);
					gpsup_flag = 1;
				}
				//复位
				else if(strcmp(cArray2,"reset") == 0)
				{
					gpsup_flag = 0;
					boardctl(GPRS_RST, 0);
					printf("reset ok\n");
				}
			}
		}
	}
	    
errout:
  g_gprs_started = false;

  printf("gprs: Terminating\n");
  return EXIT_FAILURE;
}







/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * gprs_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gprs_main(int argc, FAR char *argv[])
#endif
{
  int ret;

  printf("gprs_main: Starting the gprs_main\n");
  if (g_gprs_started)
    {
      printf("gprs_main: gprs_main already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("gprs", CONFIG_EXAMPLES_GPRS_PRIORITY,
                    CONFIG_EXAMPLES_GPRS_STACKSIZE, gprs,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("gprs_main: ERROR: Failed to start gprs: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("gprs_main: gprs started\n");
  while(1)
  {
  		sleep(1);
		//printf("gprs_main: running\n");
  }
  return EXIT_SUCCESS;
}
