/****************************************************************************
 * examples/gpio/gpio_main.c
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

#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>


#include <nuttx/drivers/pwm.h>
#include <nuttx/fs/ioctl.h>

/**************************************************************************
* DEBUG add by liubofei for printf
* add by liubofei 20171220
***************************************************************************/

#define __GO_DEBUG__  1
#if __GO_DEBUG__   
#define TEST_DEBUG(format,...) printf("[TEST_DEBUG: "__FILE__",%s(),Line:%04d:] "format"\n",__func__,__LINE__,##__VA_ARGS__)   
#else
#define TEST_DEBUG(format,...) 
#endif

/*
* @ This is a test program !!!
* @ add by liubofei 20171220  
*/

#define DEVICE_ID_FILE	"/mnt/tmp/device_id.txt"

//#define FLAGS O_RDWR| O_CREAT | O_TRUNC
//创建文件的权限，用户读、写、执行、组读、执行、其他用户读、执行  
//#define MODE S_IRWXU | S_IXGRP | S_IROTH | S_IXOTH


int parsed_string(char * string,char **num)
{
	char* src[10] = {string, NULL};
	int i=0;

	TEST_DEBUG("parsed_string()->string==%s\n",string);	
	
	for(i=0; num[i]=strtok(src[i!=0],"\r\n"); i++)
	{
		TEST_DEBUG("num[%d] =%s\n",i,num[i]);
	}

	return 0;
}

int ctl_device_id_file(char * device_id)
{
	int fd = -1;
	int fget_buff = -1;
	char get_device_id[64];
	char * get_num[2] = {0};

	system("mount  -t tmpfs -o tmpfs /mnt/tmp");

	sleep(1);

	system("cp /mnt/at24a/device_id.txt /mnt/tmp/device_id.txt");	

	//system("cat /mnt/at24a/device_id.txt > /mnt/tmp/device_id.txt");	

	fd = open(DEVICE_ID_FILE,O_RDWR);//O_RDWR|O_CREAT,0666
	if(fd < 0)
	{
		perror("Error: ");
		//TEST_DEBUG("OPEN ERROR\n");
		return -1;
	}

	//write(fd,device_id,16);
#if 0	
	sleep(1);
	fget_buff = fgets(get_device_id,20,fd);

	if(fget_buff < 0)
	{
		perror("Error: ");
		return -1;
	}
	TEST_DEBUG("get_device_id = %s,fget_buff=%d\n",get_device_id,fget_buff);
#endif	

#if 1
	read(fd,get_device_id,15);
	TEST_DEBUG("get_device_id = %s\n",get_device_id);
#endif
	parsed_string(get_device_id,get_num);

	
	return 0;
}

int for_to_test()
{
	int fd = -1;
	char get_device_id[16];

	fd = open("/dev/mtdblock1",O_RDWR,0777);//O_RDWR|O_CREAT,0666

	if(fd < 0)
	{
		perror("Error: ");
		//TEST_DEBUG("OPEN ERROR\n");
		return -1;
	}
	
	write(fd,"LLLL",15);
	sleep(1);
	read(fd,get_device_id,128);
	TEST_DEBUG("get_device_id = %s\n",get_device_id);
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int liu_test_main(int argc, char *argv[])
#endif
{
	char get_id[64] = {0};
	//ctl_device_id_file(get_id);
for_to_test();
	return 0;
}
