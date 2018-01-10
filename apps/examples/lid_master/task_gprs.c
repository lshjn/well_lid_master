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
#include <errno.h>
#include <debug.h>

#include "task_gprs.h"
#include "task_monitor.h"
#include "task_flash.h"

#define MY_READ 1

/****************************************************************************
 * Private Data
 ****************************************************************************/
static	bool		g_gprs_started;
struct	gprs_data	GprsData;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
int  fd_gprs;
/****************************************************************************
 * gprs_warn_upload
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_warn_upload(int fd,struct gprs_data *gprs,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	 	//getSystime();	//close by liubofei 2017-12-29 �ظ�����
		memset(gprs->msgbuf, 0, sizeof(gprs->msgbuf));
		/*sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=warnupload#requestok;locker=%s;mb=%.2f;tempretrue=%.2f;humidity=%.2f;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.2f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
								sensor->lockstr,adc_dada->VCC,adc_dada->tempretrue,adc_dada->humidity,
								"NULL","NULL","NULL","NULL",adc_dada->Water_high,adc_dada->VCC,
								DEV_ID);
		*///changed by liubofei 2017-12-26
		sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=warnupload#requestok;locker=%s;mb=%.1f;tempretrue=%.1f;humidity=%.1f;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.1f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
								sensor->lockstr,(int)(adc_dada->VCC_middle*10)/10.0,(int)(adc_dada->tempretrue*10)/10.0,(int)(adc_dada->humidity*10)/10.0,
								"NULL","NULL","NULL","NULL",(int)(adc_dada->Water_high*100)/100.0,(int)(adc_dada->VCC_middle*10)/10.0,
								DEV_ID);
		gprs->msglen = strlen(gprs->msgbuf);

		iBytes = write(fd,gprs->msgbuf,gprs->msglen);
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		sleep(4);
		if(cnt++ >= 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}

/****************************************************************************
 * gprs_timeint_upload
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_timeint_upload(int fd,struct gprs_data *gprs,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	 	//getSystime();	//close by liubofei 2017-12-29 �ظ�����
		memset(gprs->msgbuf, 0, sizeof(gprs->msgbuf));
		/*sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=timingupload#requestok;locker=%s;mb=%.2f;tempretrue=%.2f;humidity=%.2f;co=%.2f;h2s=%.2f;nh3=%.2f;o2=%.2f;water=%.2f;sb=%.2f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
								sensor->lockstr,adc_dada->VCC,adc_dada->tempretrue,adc_dada->humidity,
								adc_dada->CO,adc_dada->H2S,adc_dada->NH3,adc_dada->O2,adc_dada->Water_high,adc_dada->VCC,
								DEV_ID);
		*///changed by liubofei 2017-12-26
		sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=timingupload#requestok;locker=%s;mb=%.1f;tempretrue=%.1f;humidity=%.1f;co=%.1f;h2s=%.1f;nh3=%.1f;o2=%.1f;water=%.2f;sb=%.1f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
								sensor->lockstr,(int)(adc_dada->VCC_middle*10)/10.0,(int)(adc_dada->tempretrue*10)/10.0,(int)(adc_dada->humidity*10)/10.0,
								(int)(adc_dada->CO*10)/10.0,(int)(adc_dada->H2S*10)/10.0,(int)(adc_dada->NH3*10)/10.0,(int)(adc_dada->O2*10)/10.0,(int)(adc_dada->Water_high*100)/100.0,(int)(adc_dada->VCC_middle*10)/10.0,
								DEV_ID);
		
		gprs->msglen = strlen(gprs->msgbuf);
			
		iBytes = write(fd,gprs->msgbuf,gprs->msglen);
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
		sleep(4);
		if(cnt++ >= 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}

/****************************************************************************
 * gprs_openlock
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_openlock(int fd,struct gprs_data *gprs,struct adc_msg *adc_dada,struct hall_sensor *sensor,char num)
{
	int  	iBytes	= 0;
	int		cnt = 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	 	//getSystime();	//close by liubofei 2017-12-29 �ظ�����
		memset(gprs->msgbuf, 0, sizeof(gprs->msgbuf));

        if(1 == num)
        {
			sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=openlock#request;locker=%s;mb=%.1f;tempretrue=%s;humidity=%s;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.1f;id=%3d@@\n",
									DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
									DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
								  sensor->lockstr,(int)(adc_dada->VCC_middle*10)/10.0,"NULL","NULL",
								  "NULL","NULL","NULL","NULL",(int)(adc_dada->Water_high*100)/100.0,(int)(adc_dada->VCC_middle*10)/10.0,
								  DEV_ID);
	    }
	    else if(2 == num)
	    {
				sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=openlock#requestok;locker=%s;mb=%.1f;tempretrue=%s;humidity=%s;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.1f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
							  sensor->lockstr,(int)(adc_dada->VCC_middle*10)/10.0,"NULL","NULL",
							  "NULL","NULL","NULL","NULL",(int)(adc_dada->Water_high*100)/100.0,(int)(adc_dada->VCC_middle*10)/10.0,
							  DEV_ID);
	    }

		gprs->msglen = strlen(gprs->msgbuf);
		iBytes = write(fd,gprs->msgbuf,gprs->msglen);
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
		//wait
		sleep(8);
		if(cnt++ >= 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;


}


/****************************************************************************
 * gprs_openlock
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_closelock(int fd,struct gprs_data *gprs,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int  	iBytes	= 0;
	int		cnt = 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	 //	getSystime();	//close by liubofei 2017-12-29 �ظ�����
		memset(gprs->msgbuf, 0, sizeof(gprs->msgbuf));
		/*sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=closelock#requestok;locker=%s;mb=%.2f;tempretrue=%s;humidity=%s;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.2f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
							  sensor->lockstr,adc_dada->VCC,"NULL","NULL",
							  "NULL","NULL","NULL","NULL",adc_dada->Water_high,adc_dada->VCC,
							  DEV_ID);
		*///changed by liubofei 2017-12-26
		sprintf(gprs->msgbuf,"##time=%4d%d%d%d%d%d%d%d%d%d%d;msgtype=closelock#requestok;locker=%s;mb=%.1f;tempretrue=%s;humidity=%s;co=%s;h2s=%s;nh3=%s;o2=%s;water=%.2f;sb=%.1f;id=%3d@@\n",
								DisLocalTime.Year,DisLocalTime.Month/10,DisLocalTime.Month%10,DisLocalTime.Day/10,DisLocalTime.Day%10,
								DisLocalTime.Hour/10,DisLocalTime.Hour%10,DisLocalTime.Minute/10,DisLocalTime.Minute%10,DisLocalTime.Second/10,DisLocalTime.Second%10,
							  sensor->lockstr,(int)(adc_dada->VCC_middle*10)/10.0,"NULL","NULL",
							  "NULL","NULL","NULL","NULL",(int)(adc_dada->Water_high*100)/100.0,(int)(adc_dada->VCC_middle*10)/10.0,
							  DEV_ID);
		gprs->msglen = strlen(gprs->msgbuf);
		iBytes = write(fd,gprs->msgbuf,gprs->msglen);
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
		//wait
		sleep(4);
		if(cnt++ >= 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;


}


/****************************************************************************
 * set_sconType
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_sconType(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	    iBytes = write(fd,"AT^SICS=0,conType,GPRS0\r\n",strlen("AT^SICS=0,conType,GPRS0\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * set_apn
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_apn(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
		//changed by liubofei 2017-12-28
	    iBytes = write(fd,"AT^SICS=0,apn,cment\r\n",strlen("AT^SICS=0,apn,cment\r\n"));//china mobile
	    //iBytes = write(fd,"AT^SICS=0,apn,XZ01.NJM2MAPN\r\n",strlen("AT^SICS=0,apn,XZ01.NJM2MAPN\r\n"));//china mobile
	    //iBytes = write(fd,"AT^SICS=0,apn,uninet\r\n",strlen("AT^SICS=0,apn,uninet\r\n"));//china unicom
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * set_srvType
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_srvType(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	    iBytes = write(fd,"AT^SISS=0,srvType,Transparent\r\n",strlen("AT^SISS=0,srvType,Transparent\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * set_srvType
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_srvType_socket(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	    iBytes = write(fd,"AT^SISS=0,srvType,Socket\r\n",strlen("AT^SISS=0,srvType,Socket\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * set_conId
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_conId(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	    iBytes = write(fd,"AT^SISS=0,conId,0\r\n",strlen("AT^SISS=0,conId,0\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * set_address
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  set_address(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != ACK)
	{
	    //iBytes = write(fd,"AT^SISS=0,address,socktcp://47.95.250.14:8000\r\n",strlen("AT^SISS=0,address,socktcp://47.95.250.14:8000\r\n"));//for test
	    iBytes = write(fd,"AT^SISS=0,address,socktcp://220.249.21.130:1153\r\n",strlen("AT^SISS=0,address,socktcp://220.249.21.130:1153\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}

/****************************************************************************
 * into_establish
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  into_establish(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
    iBytes = write(fd,"AT^SISO=0\r\n",strlen("AT^SISO=0\r\n"));
	if(iBytes == -1)
	{
		printf("Error:write  Data to gprs\n");
	}
	//while(gprs->msgack !=ACK)
	while((gprs->msgack != ACK))
	{
		printf("into_establish---> while(1)-->gprs->msgack=%d\n",gprs->msgack);
		if(gprs->msgack == RCV_ERROR)
		{
			printf("into_establish---> while(1)--RCV_ERROR\n");
			break;
		}
		usleep(1000*1000);
		if(cnt++ > 30)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
		printf("into_establish---> while(1)---2222\n");
	gprs->msgack = NOACK;
	return SUCCESS;
}

/****************************************************************************
 * into_transparent
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  into_transparent(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	printf("gprs --> into_transparent()--->AT^SIST=0\n");
	while(gprs->msgack != CONNECT)
	{
	    iBytes = write(fd,"AT^SIST=0\r\n",strlen("AT^SIST=0\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		usleep(500*1000);
		if(cnt++ > 10)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * gprs_tcpinit
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_tcpinit(int fd,struct gprs_data *initgprs)
{
	int ret;
	ret = set_sconType(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
#if 1
	ret = set_apn(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
#endif
	ret = set_srvType(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_conId(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_address(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = into_establish(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = into_transparent(fd,initgprs);	
	if(ret == FAIL)
	{
		return FAIL;
	}
	
	return SUCCESS;	
}

/****************************************************************************
 * gprs_tcpinit2
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int  gprs_tcpinit2(int fd,struct gprs_data *initgprs)
{
	int ret;
	ret = set_sconType(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_apn(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_srvType_socket(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_conId(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = set_address(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	ret = into_establish(fd,initgprs);
	if(ret == FAIL)
	{
		return FAIL;
	}
	
	return SUCCESS;	
}
/****************************************************************************
 * master_gprs
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  gprs_register(int fd,struct	gprs_data *gprs)
{
	int  	iBytes	= 0;
	int		cnt		= 0;
	
	gprs->msgack = NOACK;
	while(gprs->msgack != REGOK)
	{
	    iBytes = write(fd,"AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
		if(iBytes == -1)
		{
			printf("Error:write  Data to gprs\n");
		}
	
		sleep(1);
		if(cnt++ > 60)
		{
			gprs->msgack = NOACK;
			return FAIL;
		}
	}
	gprs->msgack = NOACK;
	return SUCCESS;
}
/****************************************************************************
 * master_gprs
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int  gprs_rst(int fd,struct	gprs_data *gprs)
{
	int  	iBytes = 0;
	int		ret;
	
	//gprs
	boardctl(BOARDIOC_GPRS_PWROFF, 0);
	usleep(500*1000);
	boardctl(BOARDIOC_GPRS_PWRON, 0);
	usleep(500*1000);
	boardctl(BOARDIOC_GPRS_WAKEUP, 0);  
	sleep(1);
	//boardctl(BOARDIOC_GPRS_RST, 0);
	
	//set  link_led  mode
    iBytes = write(fd,"AT^SSYNC=1\r\n",strlen("AT^SSYNC=1\r\n"));
	if(iBytes == -1)
	{
		printf("Error:write  Data to gprs\n");
	}

	if(gprs_register(fd,gprs) == SUCCESS)
	{
		ret = gprs_tcpinit(fd,gprs);
		if(ret == FAIL)
		{
			return FAIL;
		}
	}
	else
	{
		return FAIL;
	}
	return SUCCESS;
}


/////////////////////////////////////

/**************************************************************************
*  my_read(int fd,void *buffer,int length)
*  liubofei
*  2018-01-05
**************************************************************************/
#if 1
int my_read(int fd,char *buffer,int length) 
{
	int bytes_left = -1;
	int bytes_read = -1; 
	//char *ptr; 

	//ptr = buffer;
	bytes_left=length;

 	while(bytes_left>0)
 	{
   		//bytes_read=read(fd,buffer,length);
		bytes_read=read(fd,buffer,1);
   		if(bytes_read<0) 
   		{
   			printf("--my_read->(bytes_read<0)--errno=%d--\n",errno);
    		if((errno == EAGAIN)||(errno == EINTR))
    		{
        		bytes_read = 0;
    		}
     		else
     		{
     			printf("----(bytes_read<0)----error-else----\n");
       			return(length-bytes_left);
     		}
   		}
   		else if(bytes_read==0) 
       		break; 
    	bytes_left-=bytes_read; 
    	buffer+=bytes_read; 
 	}
	
 return (length-bytes_left) ; 
}
#endif

/**************************************************************************
*  my_read2(int fd,void *buffer,int length)
*  liubofei
*  2018-01-08
**************************************************************************/
#if 1
//int my_read2(int fd, char *buffer, int length,FAR fd_set *rfds)  
int my_read2(int fd, char *buffer, int length)
{ 

  fd_set readrfds;
  struct timeval tv;
  ssize_t nbytes;
  bool timeout;
  bool ready;
  int ret;
  int bytes_left = -1;

	bytes_left=length;
	
  /* Loop forever */

//	for (;;)
//	{

	  timeout    = false;
	  ready      = false;

      printf("select_listener: Calling select()\n");

      FD_ZERO(&readrfds);
      FD_SET(fd, &readrfds);

      tv.tv_sec  = 60;
      tv.tv_usec = 0;

      timeout    = false;
      ready      = false;

	  ret = select(fd+1, (FAR fd_set*)&readrfds, (FAR fd_set*)NULL, (FAR fd_set*)NULL, &tv);
	  printf("\nselect_listener: select returned: %d\n", ret);

	  if (ret < 0)
	    {
	      printf("select_listener: ERROR select failed: %d\n", errno);
	    }
	  else if (ret == 0)
	    {
	      printf("select_listener: Timeout\n");
	      timeout = true;
	    }
	  else
	    {
	      if (ret != 1)
	        {
	          printf("select_listener: ERROR poll reported: %d\n", ret);
	        }
	      else
	        {
	          ready = true;
	        }

	      if (!FD_ISSET(fd, &readrfds))
	        {
	          printf("select_listener: ERROR fd=%d not in fd_set\n", fd);
			  return -1;
	        }
	    }

	  /* In any event, read until the pipe is empty */

	  do
	    {
	      nbytes = read(fd, buffer, length);
	      if (nbytes <= 0)
	        {
	          if (nbytes == 0 || errno == EAGAIN)
	            {
	              if (ready)
	                {
	                  printf("select_listener: ERROR no read data\n");
	                }
	            }
	          else if (errno != EINTR)
	            {
	              printf("select_listener: read failed: %d\n", errno);
	            }
	          nbytes = 0;
	        }
	      else
	        {
	          if (timeout)
	            {
	              printf("select_listener: ERROR? Poll timeout, but data read\n");
	              printf("               (might just be a race condition)\n");
	            }

	          //buffer[nbytes] = '\0';
	          printf("select_listener: Read '%s' (%ld bytes)\n", buffer, (long)nbytes);
	        }

    		bytes_left-=nbytes; 
    		buffer+=nbytes; 

	      timeout = false;
	      ready   = false;
	    }
	  while (nbytes > 0);

	  /* Make sure that everything is displayed */

	  fflush(stdout);
//	}
	return (length-bytes_left) ;
} 
#endif
/////////////////////////////////////


/****************************************************************************
 * master_gprs
 * liushuhe
 * 2017.10.10
 ****************************************************************************/
int master_gprs(int argc, char *argv[])
{
	struct timeval timeout;
	fd_set 	rfds;	
	int		fd_rtc;
	
    char   *pcTempBuf[200];
    char   *pChar = ",";
    char   *pChar2 = ";";
	int		cCharNum = 0;

	char 	cArray[200];
	char 	rcvmsg[200];
	
	int  	iRet = 0;
	int  	iBytes = 0;
	int  	rcvmsg_ok = 0;
	
	GprsData.msgack		= NOACK;
	
	g_gprs_started = true;


	fd_gprs = open(CONFIG_EXAMPLES_GPRS_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_gprs < 0)
	{
		int errcode = errno;
		printf("gprs: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_GPRS_DEVPATH, errcode);
		goto errout;
	}

	fd_rtc = open(CONFIG_EXAMPLES_RTC_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_rtc < 0)
	{
		int errcode = errno;
		printf("rtc0: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_RTC_DEVPATH, errcode);
	}

	/*
	//gprs power
	boardctl(BOARDIOC_GPRS_PWRON, 0);
	usleep(100*1000);
	boardctl(BOARDIOC_GPRS_WAKEUP, 0);  
	usleep(100*1000);
	printf("gprs power on!!!\n");
	*/
	while(1)
	{
	#ifndef MY_READ
		FD_ZERO(&rfds);											
		FD_SET(fd_gprs, &rfds);									
		timeout.tv_sec = 60;
		timeout.tv_usec = 0;
		iRet = select(fd_gprs+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			//printf("gprs_dev rcv timeout!!!\n");
		}
	 #endif
	 #ifndef MY_READ
		else
	 #endif
		{
		#ifndef MY_READ
			if(FD_ISSET(fd_gprs, &rfds))
		#endif
			{
				usleep(300*1000L);                                     //sleep 100ms
				memset(cArray, 0, sizeof(cArray));
				//iBytes = read(fd_gprs, cArray, sizeof(cArray));
				//iBytes = my_read(fd_gprs, cArray, sizeof(cArray));
				iBytes = my_read2(fd_gprs, cArray, sizeof(cArray));
				
				printf("READ_FILE-->rcv gprs <%d> bytes msg:%s\n",iBytes,cArray);
				
				if(iBytes == -1)
				{
					printf("Error:read  Data to gprs\n");
				}
			    tcflush(fd_gprs, TCIFLUSH);
				/*************************************************************************************/
				//deal  server ack


				if(strstr(cArray,"#") != NULL)
				{
					sprintf(rcvmsg,"%s",cArray);
					rcvmsg_ok = START;
				}
				if(strstr(rcvmsg,"#") != NULL)
				{
					if(rcvmsg_ok == START)
					{
						rcvmsg_ok = WAIT;
					}
					else
					{
						strcat(rcvmsg,cArray);
					}
					//rcv ok
					if(strstr(rcvmsg,"@@") != NULL)
					{
						rcvmsg_ok = RCV_OK;
						sprintf(GprsData.download_data.time,"%s",rcvmsg);
						GprsData.set_slavetime_flag = 1;
					}
				}
				/*************************************************************************************/
				printf("rcv gprs <%d> bytes msg:%s\n",iBytes,cArray);
 				/************************************************************************************/
				if(strstr(cArray,"+CREG:") != NULL)
				{
					cCharNum = 0;
					memset(pcTempBuf, 0, sizeof(pcTempBuf));
					pcTempBuf[0] = strtok((char*)cArray, pChar);
					while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//�ֽ��ַ���
					{
						usleep(1000);
					}
					//get data
					char  CREG = pcTempBuf[1][0];
					if((CREG == '1')||(CREG == '5'))
					{
						GprsData.msgack     = REGOK;
					}
				}
				else if(strstr(cArray,"CONNECT") != NULL)
				{
					GprsData.msgack     = CONNECT;
					printf("GPRS TCP:CONNECT OK\n");
				}
				else if(strstr(cArray,"OK") != NULL)
				{
					GprsData.msgack     = ACK;
					printf("GPRS Return\"OK\" \n");
				}
				//new add by liubofei 2017-12-28
				else if(strstr(cArray,"ERROR") != NULL)
				{
					GprsData.msgack     = RCV_ERROR;
					printf("GPRS Return\"ERROR\" \n");
				}
				
				//gprs server ack
				if(rcvmsg_ok == RCV_OK)
				{
					printf("server ack msg:%s\n",rcvmsg);
					if(strstr(rcvmsg,"##time=") != NULL)
					{
						cCharNum = 0;
						memset(pcTempBuf, 0, sizeof(pcTempBuf));
						pcTempBuf[0] = strtok((char*)rcvmsg, pChar2);
						while((pcTempBuf[++cCharNum] = strtok(NULL, pChar2))!= NULL)  																																											//�ֽ��ַ���
						{
							usleep(1000);
						}
						int i=0;
						for(i=0;i<cCharNum;i++)
						{
							//lock state
							if(strstr(pcTempBuf[i],"locker=on") != NULL)
							{
								GprsData.download_data.locker = ENABLE;
							}
							else if(strstr(pcTempBuf[i],"locker=off") != NULL)
							{
								GprsData.download_data.locker = DISABLE;
							}
						}
						GprsData.msgack     = ACK;
						//set time
						for(i=0;i<cCharNum;i++)
						{
							if(strstr(pcTempBuf[i],"##time=") != NULL)
							{
	   							rtctime.tm_year	= ((pcTempBuf[i][7]-0x30)*1000 + (pcTempBuf[i][8]-0x30)*100 +
												  (pcTempBuf[i][9]-0x30)*10   + (pcTempBuf[i][10]-0x30))-1900;
	   							rtctime.tm_mon 	= ((pcTempBuf[i][11]-0x30)*10   + (pcTempBuf[i][12]-0x30)) - 1;						
	   							rtctime.tm_mday	= (pcTempBuf[i][13]-0x30)*10   + (pcTempBuf[i][14]-0x30);
	   							rtctime.tm_hour	= ((pcTempBuf[i][15]-0x30)*10   + (pcTempBuf[i][16]-0x30));
	   							rtctime.tm_min	= (pcTempBuf[i][17]-0x30)*10   + (pcTempBuf[i][18]-0x30);
	   							rtctime.tm_sec	= (pcTempBuf[i][19]-0x30)*10   + (pcTempBuf[i][20]-0x30);

								printf("gprs:%d:%d:%d:%d:%d:%d\n",rtctime.tm_year,rtctime.tm_mon,rtctime.tm_mday,
													  rtctime.tm_hour,rtctime.tm_min,rtctime.tm_sec);
								setRtcTime(fd_rtc,&rtctime);
								getRtcTime(fd_rtc,&rtctime);
								setSystime(&rtctime);
							}
						}
					}
					memset(rcvmsg, 0, sizeof(rcvmsg));
				}
				
			}
		}
	}
	    
errout:
  g_gprs_started = false;

  printf("gprs: Terminating\n");
  return EXIT_FAILURE;
}

