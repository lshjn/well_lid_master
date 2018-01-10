#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>


#include "task_monitor.h"
#include "task_flash.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct alarm_value  alarmdata;

/****************************************************************************
 * mid
 * liushuhe
 * 2017.10.23
 ****************************************************************************/
char * mid(char *dst,char *src, int n,int m) /*n为长度，m为位置*/  
{  
    char *p = src;  
    char *q = dst;  
    int len = strlen(src);  
    if(n>len) n = len-m;    /*从第m个到最后*/  
    if(m<0) m=0;    /*从第一个开始*/  
    if(m>len) return NULL;  
    p += m;  
    while(n--) *(q++) = *(p++);  
    *(q++)='\0'; /*有必要吗？很有必要*/  
    return dst;  
}  

/****************************************************************************
 * analyflashdata1
 * liushuhe
 * 2017.10.23
 ****************************************************************************/
void analyflashdata1(char **pcTempBuf,int cCharNum)
{
    char   TempBuf[100];
    int    i = 0;
    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"settime=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-8),8);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.timingA_hour = SET_TIMEA_HOUR;
				alarmdata.timingA_min  = SET_TIMEA_MIN;
				alarmdata.timingB_hour = SET_TIMEB_HOUR;
				alarmdata.timingB_min  = SET_TIMEB_MIN;
			}
			else
			{
			    if((TempBuf[0] >='0')&&(TempBuf[0] <='9')&&(TempBuf[1] >='0')&&(TempBuf[1] <='9')&&
				  (TempBuf[3] >='0')&&(TempBuf[3] <='9')&&(TempBuf[4] >='0')&&(TempBuf[4] <='9')&&
				  (TempBuf[6] >='0')&&(TempBuf[6] <='9')&&(TempBuf[7] >='0')&&(TempBuf[7] <='9')&&
				  (TempBuf[9] >='0')&&(TempBuf[9] <='9')&&(TempBuf[10] >='0')&&(TempBuf[10] <='9'))
			    {
					alarmdata.timingA_hour = (TempBuf[0]-0x30)*10+(TempBuf[1]-0x30);
					alarmdata.timingA_min  = (TempBuf[3]-0x30)*10+(TempBuf[4]-0x30);
					alarmdata.timingB_hour = (TempBuf[6]-0x30)*10+(TempBuf[7]-0x30);
					alarmdata.timingB_min  = (TempBuf[9]-0x30)*10+(TempBuf[10]-0x30);
				}
				else
				{
					alarmdata.timingA_hour = SET_TIMEA_HOUR;
					alarmdata.timingA_min  = SET_TIMEA_MIN;
					alarmdata.timingB_hour = SET_TIMEB_HOUR;
					alarmdata.timingB_min  = SET_TIMEB_MIN;
				}
			}
			printf("timingA=%2d:%2d,timingB=%2d:%2d\n",alarmdata.timingA_hour,alarmdata.timingA_min,alarmdata.timingB_hour,alarmdata.timingB_min);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.timingA_hour = SET_TIMEA_HOUR;
		alarmdata.timingA_min  = SET_TIMEA_MIN;
		alarmdata.timingB_hour = SET_TIMEB_HOUR;
		alarmdata.timingB_min  = SET_TIMEB_MIN;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"mb=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.vcc_mb = VCC_MB_DEF;
			}
			else
			{
				alarmdata.vcc_mb = atof(TempBuf);
			}
			printf("mb= %s,%.2f\n",TempBuf,alarmdata.vcc_mb);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.vcc_mb = VCC_MB_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"tempretrue=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-11),11);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.tempretrue = TEMPRETURE_DEF;
			}
			else
			{
				alarmdata.tempretrue = atof(TempBuf);
			}
			printf("tempretrue= %s,%.2f\n",TempBuf,alarmdata.tempretrue);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.tempretrue = TEMPRETURE_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"humidity=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-9),9);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.humidity = HUMIDITY_DEF;
			}
			else
			{
				alarmdata.humidity = atof(TempBuf);
			}
			printf("humidity= %s,%.2f\n",TempBuf,alarmdata.humidity);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.humidity = HUMIDITY_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"co=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.co = CO_DEF;
			}
			else
			{
				alarmdata.co = atof(TempBuf);
			}
			printf("co= %s,%.2f\n",TempBuf,alarmdata.co);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.co = CO_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"h2s=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-4),4);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.h2s = H2S_DEF;
			}
			else
			{
				alarmdata.h2s = atof(TempBuf);
			}
			printf("h2s= %s,%.2f\n",TempBuf,alarmdata.h2s);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.h2s = H2S_DEF;
	}
	
    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"nh3=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-4),4);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.nh3 = NH3_DEF;
			}
			else
			{
				alarmdata.nh3 = atof(TempBuf);
			}
			printf("nh3= %s,%.2f\n",TempBuf,alarmdata.nh3);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.nh3 = NH3_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"o2=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.o2 = O2_DEF;
			}
			else
			{
				alarmdata.o2 = atof(TempBuf);
			}
			printf("o2= %s,%.2f\n",TempBuf,alarmdata.o2);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.o2 = O2_DEF;
	}

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"water=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-6),6);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				alarmdata.water = WATER_DEF;
			}
			else
			{
				alarmdata.water = atof(TempBuf);
			}
			printf("water= %s,%.2f\n",TempBuf,alarmdata.water);
            //out
            break;
		}
    }
	if(i==cCharNum)
	{
		alarmdata.water = WATER_DEF;
	}
	
}



/****************************************************************************
 * analyflashdata2
 * liushuhe
 * 2017.10.23
 ****************************************************************************/
void analyflashdata2(char **pcTempBuf,int cCharNum)
{
    char   TempBuf[100];
    int    i = 0;
    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"settime=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-8),8);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.timingA_hour = SET_TIMEA_HOUR;
				//alarmdata.timingA_min  = SET_TIMEA_MIN;
				//alarmdata.timingB_hour = SET_TIMEB_HOUR;
				//alarmdata.timingB_min  = SET_TIMEB_MIN;
			}
			else
			{
			    if((TempBuf[0] >='0')&&(TempBuf[0] <='9')&&(TempBuf[1] >='0')&&(TempBuf[1] <='9')&&
				  (TempBuf[3] >='0')&&(TempBuf[3] <='9')&&(TempBuf[4] >='0')&&(TempBuf[4] <='9')&&
				  (TempBuf[6] >='0')&&(TempBuf[6] <='9')&&(TempBuf[7] >='0')&&(TempBuf[7] <='9')&&
				  (TempBuf[9] >='0')&&(TempBuf[9] <='9')&&(TempBuf[10] >='0')&&(TempBuf[10] <='9'))
			    {
					alarmdata.timingA_hour = (TempBuf[0]-0x30)*10+(TempBuf[1]-0x30);
					alarmdata.timingA_min  = (TempBuf[3]-0x30)*10+(TempBuf[4]-0x30);
					alarmdata.timingB_hour = (TempBuf[6]-0x30)*10+(TempBuf[7]-0x30);
					alarmdata.timingB_min  = (TempBuf[9]-0x30)*10+(TempBuf[10]-0x30);
				}
				else
				{
					//alarmdata.timingA_hour = SET_TIMEA_HOUR;
					//alarmdata.timingA_min  = SET_TIMEA_MIN;
					//alarmdata.timingB_hour = SET_TIMEB_HOUR;
					//alarmdata.timingB_min  = SET_TIMEB_MIN;
				}
			}
			printf("timingA=%2d:%2d,timingB=%2d:%2d\n",alarmdata.timingA_hour,alarmdata.timingA_min,alarmdata.timingB_hour,alarmdata.timingB_min);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"mb=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.vcc_mb = VCC_MB_DEF;
			}
			else
			{
				alarmdata.vcc_mb = atof(TempBuf);
			}
			printf("mb= %s,%.2f\n",TempBuf,alarmdata.vcc_mb);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"tempretrue=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-11),11);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.tempretrue = TEMPRETURE_DEF;
			}
			else
			{
				alarmdata.tempretrue = atof(TempBuf);
			}
			printf("tempretrue= %s,%.2f\n",TempBuf,alarmdata.tempretrue);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"humidity=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-9),9);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.humidity = HUMIDITY_DEF;
			}
			else
			{
				alarmdata.humidity = atof(TempBuf);
			}
			printf("humidity= %s,%.2f\n",TempBuf,alarmdata.humidity);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"co=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.co = CO_DEF;
			}
			else
			{
				alarmdata.co = atof(TempBuf);
			}
			printf("co= %s,%.2f\n",TempBuf,alarmdata.co);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"h2s=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-4),4);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.h2s = H2S_DEF;
			}
			else
			{
				alarmdata.h2s = atof(TempBuf);
			}
			printf("h2s= %s,%.2f\n",TempBuf,alarmdata.h2s);
            //out
            break;
		}
    }
	
    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"nh3=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-4),4);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.nh3 = NH3_DEF;
			}
			else
			{
				alarmdata.nh3 = atof(TempBuf);
			}
			printf("nh3= %s,%.2f\n",TempBuf,alarmdata.nh3);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"o2=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-3),3);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.o2 = O2_DEF;
			}
			else
			{
				alarmdata.o2 = atof(TempBuf);
			}
			printf("o2= %s,%.2f\n",TempBuf,alarmdata.o2);
            //out
            break;
		}
    }

    for(i=0;i<cCharNum;i++)
    {
		if(strstr(pcTempBuf[i],"water=") != NULL)
		{
			memset(TempBuf, 0, sizeof(TempBuf));
			mid(TempBuf,pcTempBuf[i],(strlen(pcTempBuf[i])-6),6);
			if(strstr(TempBuf,"NULL") != NULL)
			{
				//alarmdata.water = WATER_DEF;
			}
			else
			{
				alarmdata.water = atof(TempBuf);
			}
			printf("water= %s,%.2f\n",TempBuf,alarmdata.water);
            //out
            break;
		}
    }
	
}

/****************************************************************************
 * slave_flash
 * liushuhe
 * 2017.10.23
 ****************************************************************************/

int master_flash(int argc, char *argv[])
{
    char   *pcTempBuf[255];
    char   *pChar = ";";
	int		cCharNum = 0;

	FAR uint32_t *buffer;
	ssize_t nbytes;
	int fd;

	buffer = (FAR uint32_t *)malloc(200);
	if (!buffer)
	{
		printf("ERROR: failed to allocate a sector buffer\n");
	}

	//check flash
	fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_RDONLY);
	if (fd < 0)
	{
		printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
	}
	nbytes = read(fd, buffer, 200);
	if (nbytes < 0)
	{
		printf("ERROR: read from /dev/mtd0 failed: %d\n", errno);
	}
	else
	{
		if((strstr((char *)buffer,"##time=") != NULL)&&(strlen((char *)buffer) > 50))
		{
			printf("read flash data:%s\n",buffer);
			cCharNum = 0;
			memset(pcTempBuf, 0, sizeof(pcTempBuf));
			pcTempBuf[0] = strtok((char*)buffer, pChar);
			while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
			{
			}
			//get alarm data
			analyflashdata1((char **)&pcTempBuf,cCharNum);
		}
		else
		{
			close(fd);
			//init flash data
			fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_WRONLY);
			if (fd < 0)
			{
				printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
			}
			
			sprintf((char*)buffer,"##time=20151020170733;settime=07:00#20:00;locker=off;mb=%.1f;tempretrue=%.1f;humidity=%.1f;co=%.1f;h2s=%.1f;nh3=%.1f;o2=%.1f;water=%.1f;sb=%.1f;id=123@@\n",
								VCC_MB_DEF,TEMPRETURE_DEF,HUMIDITY_DEF,CO_DEF,H2S_DEF,NH3_DEF,O2_DEF,WATER_DEF,VCC_SB_DEF);
			nbytes = write(fd, buffer, strlen((char *)buffer));
			if (nbytes < 0)
			{
				printf("ERROR: write to /dev/mtd0 failed: %d\n", errno);
			}

		  	close(fd);

			
			alarmdata.timingA_hour = SET_TIMEA_HOUR;
			alarmdata.timingA_min  = SET_TIMEA_MIN;
			alarmdata.timingB_hour = SET_TIMEB_HOUR;
			alarmdata.timingB_min  = SET_TIMEB_MIN;
			
			alarmdata.vcc_mb		= VCC_MB_DEF;
			alarmdata.tempretrue 	= TEMPRETURE_DEF;
			alarmdata.humidity 		= HUMIDITY_DEF;
			alarmdata.co 			= CO_DEF;
			alarmdata.h2s 			= H2S_DEF;
			alarmdata.nh3 			= NH3_DEF;
			alarmdata.o2 			= O2_DEF;
			alarmdata.water 		= WATER_DEF;
		}
	}
	close(fd);
	

	while(1)
	{
		sleep(1);
		if(GprsData.set_slavetime_flag == 1)
		{
			GprsData.set_slavetime_flag = 0;
			fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_WRONLY);
			if (fd < 0)
			{
				printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
			}
			//send time
			sprintf((char*)buffer,"%s",GprsData.download_data.time);
			nbytes = write(fd, buffer, strlen((char *)buffer));
			if (nbytes < 0)
			{
				printf("ERROR: write to %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH,errno);
			}
			if(strstr((const char *)buffer,"##time=") != NULL)
			{
				printf("refresh alarm data:%s\n",buffer);
				cCharNum = 0;
				memset(pcTempBuf, 0, sizeof(pcTempBuf));
				pcTempBuf[0] = strtok((char*)buffer, pChar);
				while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
				{
				}
				analyflashdata2((char **)&pcTempBuf,cCharNum);
			}
			close(fd);
		}
	}

  return 0;
}



