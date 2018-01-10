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

#include "task_adc.h"
#include "task_flash.h"
#include "task_485.h"
#include "task_gprs.h"
#include "task_bluetooth.h"
#include "task_monitor.h"
#include "task_motor.h"


#define NTPYear						0x16D
#define NTPDay                  	0x18
#define NTPHour                 	0x3C
#define NTPMin                 		0x3C	


/*****************************************************************************************************************************
 * Private Data
 ****************************************************************************************************************************/
pthread_mutex_t g_TimIntMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_MonitorMutex		= PTHREAD_MUTEX_INITIALIZER;

static		bool  		g_monitor_started;

char		TimeInt_SampleFlag = 0;


struct  	TimeStruct  DisLocalTime;
struct		tm 			*tmp;
struct 		rtc_time	rtctime;
time_t		timelocal;

struct  	TimeStruct_check  time_init;

///////////////////////////////
//new add by liubofei 2017-12-29

/*********************************************************************************************************************************
函数名称：int leapdays(int styear, int year)
功    能：计算闰年天数
输入参数：
输出参数：
编写时间：2016.06.15
编 写 人：lsh
注    意：
*********************************************************************************************************************************/
int leapdays(int styear, int year)
{
    int yt = styear - 1,leapday = 0;
    while(year > yt)
    {
        yt++;
        if(yt % 400 == 0)
        {
            leapday++;
            continue;
        }
        else if(yt % 100 == 0)
        {
            continue;
        }
        else if(yt % 4 == 0)
        {
            leapday++;
        }
    }
    return leapday;
}


/*****************************************************************************************************
函数名称：DealIntegerTime
功		能：军标时间 计算秒信息到日期
输入参数：无
输出参数：无
编写时间：2016.03.24
作		者：lsh
******************************************************************************************************/
void DealIntegerTime(unsigned long it , struct TimeStruct *time,int styear)
{
    int monthh[12] = {31 , 28 , 31 , 30 , 31 , 30 , 31 , 31 , 30 , 31 , 30 , 31};
    int year = 0,month = 1,day = 0,hour = 0,minute = 0,second = 0,leapday = 0;
    unsigned long t;
    int i = 0;
    year = styear + it / 31536000;
    t = it % 31536000;
    day = t / 86400 +1;
    if((year%400==0)||(year % 4 == 0&&year%100!=0))
    {
        day++;
    }
    t = t % 86400;
    leapday = leapdays(styear,year);
    if(leapday >= day)
    {
        year--;
        day = 365 + day - leapday;
        if((year%400==0)||(year % 4 == 0&&year%100!=0)) day++;
    }
    else
    {
        day = day - leapday;
    }
    hour =  t / 3600;
    t = t % 3600 ;
    minute = t / 60;
    second = t % 60;

    while(day > monthh[i] + ((i == 1 && !(year % 4)) ? 1:0))
    {
        day = day - monthh[i] - ((i == 1 && !(year % 4)) ? 1:0);
        month++;
        i++;
    }
    time->Year = year ;
    time->Month = month;
    time->Day = day;
    time->Hour =  hour;
    time->Minute = minute;
    time->Second = second;
}

/*****************************************************************************************************
函数名称：CalcSecond_BdGps
功		能：将RMC报文转换为NTP秒
输入参数：无
输出参数：无
编写时间：2016.06.15
作		者：lsh
******************************************************************************************************/
void CalcSecond_BdGps(struct TimeStruct_check *time,int styear)
{
//time->NTPSecond = 0;
    time->NTPSecond = ((time->Year - styear)* 365 + leapdays(styear,(time->Year-1)))*NTPDay*NTPHour*NTPMin;
    if (time->Month == 1)
    {
        time->NTPSecond = time->NTPSecond;
    }
    else if  (time->Month == 2)
    {
        time->NTPSecond = time->NTPSecond + 31 *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 3 )
    {
        time->NTPSecond = time->NTPSecond + (31 + 28) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 4)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 5)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31+30) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 6)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 +31) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 7)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 8)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30 + 31) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 9)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30 + 31 + 31) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 10)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30 + 31 + 31 + 30) *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 11)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30 + 31 + 31 + 30 +31)  *NTPDay*NTPHour*NTPMin;
    }
    else if  (time->Month == 12)
    {
        time->NTPSecond = time->NTPSecond + (31 +28 +31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30) *NTPDay*NTPHour*NTPMin;
    }
    if ((time->Year%4 == 0) && time->Month>2)
    {
        time->NTPSecond = time->NTPSecond + NTPDay*NTPHour*NTPMin;
    }
    time->NTPSecond = time->NTPSecond	+(time->Day - 1) *NTPDay*NTPHour*NTPMin\
                        + time->Hour*NTPHour*NTPMin + time->Minute*NTPMin + time->Second ;
   // time->NTPSecond = time->NTPSecond + 8*3600;
   /*if(DisLocalTime.system_init == 0)
   {
   		time->NTPSecond_tmp = time->NTPSecond;
   }
	*/
}

//end
//////////////////////////////


/****************************************************************************
 * getSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
void  getSystime(void)
{
	time(&timelocal);	
	tmp = localtime(&timelocal);  //获取本地时间
//changed by liubofei
/*
	DisLocalTime.Year		=	1900+tmp->tm_year;
	DisLocalTime.Month		=	tmp->tm_mon + 1;
	DisLocalTime.Day		=	tmp->tm_mday;
	DisLocalTime.Hour		=	tmp->tm_hour;
	DisLocalTime.Minute	=	tmp->tm_min;
	DisLocalTime.Second	=	tmp->tm_sec;
	*/

	time_init.Year		=	1900+tmp->tm_year;
	time_init.Month		=	tmp->tm_mon + 1;
	time_init.Day		=	tmp->tm_mday;
	time_init.Hour		=	tmp->tm_hour;
	time_init.Minute	=	tmp->tm_min;
	time_init.Second	=	tmp->tm_sec;
	
}

/////////////////////////////////////////////////////////
/************************************************
* new add by liubofei 2017-12-29
* getSystime_check () 时间异常值过滤
*/
void  getSystime_check(void)
{
	getSystime();
		
	//printf("--getSystime_check(void)-->DisLocalTime.system_init=%d\n",DisLocalTime.system_init);

	if(DisLocalTime.system_init == 0)//设备第一次开机
	{
		
		CalcSecond_BdGps(&time_init,1900);
		
		time_init.NTPSecond_tmp = time_init.NTPSecond;
		////DealIntegerTime(time_init.NTPSecond_check,&DisLocalTime,1900);
		
		DisLocalTime.system_init = 1;
	}
	else
	{
		CalcSecond_BdGps(&time_init,1900);

		//printf("time_init.NTPSecond_tmp= %u\ntime_init.NTPSecond=%u\n",time_init.NTPSecond_tmp ,time_init.NTPSecond);
	
		if((time_init.NTPSecond_tmp > 0) && (time_init.NTPSecond > 0))
		{
			//if(time_init.NTPSecond - time_init.NTPSecond_tmp == 1)
			////if((abs (time_init.NTPSecond - time_init.NTPSecond_tmp) < 60) )
			if(((time_init.NTPSecond - time_init.NTPSecond_tmp) < 60) && ((time_init.NTPSecond - time_init.NTPSecond_tmp) > 0))
			{
				time_init.NTPSecond_check = time_init.NTPSecond;
				time_init.NTPSecond_tmp  = time_init.NTPSecond;

				DealIntegerTime(time_init.NTPSecond_check,&DisLocalTime,1900);
				
			}
			else if(0 == (time_init.NTPSecond - time_init.NTPSecond_tmp))//因为500ms取一次值，NTPSecond 是按秒计算的，会有重复值 
			{
			}
			else
			{
				printf("----ERROR---- [ time_init.NTPSecond  ]\n");
				DisLocalTime.system_init = 0;//异常后置为 0 ，重新获取新数据存放到变量里
				////time_init.NTPSecond ++;
			}
			printf("LIUBOFEI-->time:%d-%d-%d-%d-%d-%d\n",DisLocalTime.Year,DisLocalTime.Month,DisLocalTime.Day,
						  DisLocalTime.Hour,DisLocalTime.Minute,DisLocalTime.Second);
		}
	}
	
}
///////////////////////////////////////////////////////////

/****************************************************************************
 * setSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int setSystime(struct rtc_time *rtc)
{
	//struct rtc_time rtc;  
	struct tm _tm;  
	struct timeval tv;  
	time_t timep;  

	_tm.tm_sec 	= rtc->tm_sec;  
	_tm.tm_min 	= rtc->tm_min;  
	_tm.tm_hour = rtc->tm_hour;  
	_tm.tm_mday = rtc->tm_mday;  
	_tm.tm_mon 	= rtc->tm_mon - 1;  
	_tm.tm_year = rtc->tm_year - 1900;  

	timep = mktime(&_tm);  
	tv.tv_sec = timep;  
	tv.tv_usec = 0;  
	if(settimeofday (&tv, (struct timezone *) 0) < 0)  
	{  
		printf("Set system datatime error!/n");  
		return -1;  
	}
	
	return 0;  

}
/****************************************************************************
 * setRtcTime
 * liushuhe
 * 2017.10.18
 ****************************************************************************/
int setRtcTime(int fd,struct rtc_time *rtc)
{
	int ret;
	ret = ioctl(fd, RTC_SET_TIME,(unsigned long)((uintptr_t)rtc));
	if (ret < 0)
	{
		int errcode = errno;
		printf("getRtcTime: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",errcode);
		return -1;
	}
	return 1;

}
/****************************************************************************
 * getRtcTime
 * liushuhe
 * 2017.10.18
 ****************************************************************************/
int getRtcTime(int fd,struct rtc_time *rtc)
{
	int ret;
	ret = ioctl(fd, RTC_RD_TIME,(unsigned long)((uintptr_t)rtc));
	if (ret < 0)
	{
		int errcode = errno;
		printf("getRtcTime: ERROR: ioctl(ULEDIOC_SUPPORTED) failed: %d\n",errcode);
		return -1;
	}

	//year + 1900
	rtc->tm_year = rtc->tm_year + 1900;
	//tm_mon + 1
	rtc->tm_mon = rtc->tm_mon + 1;


	
	return 1;
}

/******************************************************************************************************************************
函数名称：int	CheckTimeInt(void)
功    能：判断是否到达定时采集时间
输入参数：无
输入参数：1:使能	0:禁止
编写时间：2017.09.29
编 写 人：liushuhe
*******************************************************************************************************************************/
int	CheckTimeInt(void)
{
	//获取本地时间
	//getSystime();//close by liubofei 2017-12-29 重复调用
	
	printf("time:%d-%d-%d-%d-%d-%d\n",DisLocalTime.Year,DisLocalTime.Month,DisLocalTime.Day,
						  DisLocalTime.Hour,DisLocalTime.Minute,DisLocalTime.Second);
		printf("alarmdata.timingA:%d-%d,alarmdata.timingB:%d-%d\n",alarmdata.timingA_hour,alarmdata.timingA_min,alarmdata.timingB_hour,
						  alarmdata.timingB_min);
	if(((0 == alarmdata.timingA_hour)&&(0 == alarmdata.timingA_min)) || ((0 == alarmdata.timingB_hour)&&(0 == alarmdata.timingB_min)))
	{
		return 0;	
	}
	else if(((alarmdata.timingA_hour == DisLocalTime.Hour)&&(alarmdata.timingA_min == DisLocalTime.Minute)&&(0 == DisLocalTime.Second))||
	   ((alarmdata.timingB_hour == DisLocalTime.Hour)&&(alarmdata.timingB_min == DisLocalTime.Minute)&&(0 == DisLocalTime.Second)))
	{
		return	1;
	}
	else
	{
		return 0;
	}
}


/****************************************************************************
 * warn_upload_process
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
void warn_upload_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int ret = FAIL;
	int cnt = 0;
//changed by liubofei 2017-12-26 (check lockstate --> report data )
	int fd_sensora;
	int fd_sensorb;
	int cnt_lock = 0;
	*locker = ASK_OPENLOCK;

	fd_sensora = open(CONFIG_EXAMPLES_SENSORA_DEVPATH, O_RDONLY);
	if (fd_sensora < 0)
	{
		printf("fd_sensora: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errno);
	}

	fd_sensorb = open(CONFIG_EXAMPLES_SENSORB_DEVPATH, O_RDONLY);
	if (fd_sensorb < 0)
	{
		printf("fd_sensorb: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errno);
	}

	while(strcmp(sensor->lockstr,"on"))
	{
		check_lockstate(fd_sensora,fd_sensorb,&Hall_Sensor);
		usleep(500*1000);//0.5s
		if(cnt_lock++ > 60)
		{
				cnt_lock = 0;
			//	close(fd_sensora);
			//	close(fd_sensorb);
			break;
		}
	}
	close(fd_sensora);
	close(fd_sensorb);
//end	
	while(ret == FAIL)
	{
		ret = gprs_rst(fd,gprsdata);
		if(cnt++ > 3)
		{
			cnt = 0;
			Gprsfail_buzz_alarm();			
			break;
		}
	}
		
	if(ret == SUCCESS)
	{
		ret = gprs_warn_upload(fd,gprsdata,adc_dada,sensor);
		if(ret == SUCCESS)
		{
			printf("warn upload ok\n");
			gprsdata->process_state = SUCCESS;
		}
		else if(ret == FAIL)
		{
			printf("warn upload fail\n");
			gprsdata->process_state = FAIL;
		}
	}
	else
	{
		//ask open lock
		printf("gprs connect fail\n");
	}
	//ask open lock
//	*locker = ASK_OPENLOCK;//closed by liubofei
}

/****************************************************************************
 * timeint_upload_process
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
void timeint_upload_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int ret = FAIL;
	int cnt = 0;
//changed by liubofei 2017-12-26 (check lockstate --> report data )
	int fd_sensora;
	int fd_sensorb;
	int cnt_lock = 0;
	*locker = ASK_CLOSELOCK;

	fd_sensora = open(CONFIG_EXAMPLES_SENSORA_DEVPATH, O_RDONLY);
	if (fd_sensora < 0)
	{
		printf("fd_sensora: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errno);
	}

	fd_sensorb = open(CONFIG_EXAMPLES_SENSORB_DEVPATH, O_RDONLY);
	if (fd_sensorb < 0)
	{
		printf("fd_sensorb: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errno);
	}

	while(strcmp(sensor->lockstr,"off"))
	{
		check_lockstate(fd_sensora,fd_sensorb,&Hall_Sensor);
		usleep(500*1000);//0.5s
		if(cnt_lock++ > 60)
		{
				cnt_lock = 0;
				//close(fd_sensora);
				//close(fd_sensorb);
				break;
		}
	}
	close(fd_sensora);
	close(fd_sensorb);
//end
	
	while(ret == FAIL)
	{
		ret = gprs_rst(fd,gprsdata);
		if(cnt++ > 3)
		{
			cnt = 0;
			Gprsfail_buzz_alarm();			
			break;
		}
	}

	if(ret == SUCCESS)
	{
		ret = gprs_timeint_upload(fd,gprsdata,adc_dada,sensor);
		if(ret == SUCCESS)
		{
			printf("timeint upload ok\n");
			if(gprsdata->download_data.locker == ENABLE)
			{
				//定时上报只能关锁，不能开锁
				//ask open lock
				//*locker = ASK_OPENLOCK;
			}
			else if(gprsdata->download_data.locker == DISABLE)
			{
				//ask close lock
				*locker = ASK_CLOSELOCK;
			}
			gprsdata->process_state = SUCCESS;
		}
		else
		{
			//ask close lock
			*locker = ASK_CLOSELOCK;
			printf("timeint upload fail\n");
			gprsdata->process_state = FAIL;
		}
	}
	else
	{
		//ask close lock
		*locker = ASK_CLOSELOCK;
		printf("gprs connect fail\n");
	}
}
/****************************************************************************
 * w315mhz_ask_openlock_process
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
void w315mhz_ask_openlock_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{

	int fd_sensora;
	int fd_sensorb;

	int ret = FAIL;
	int cnt = 0;
	
	fd_sensora = open(CONFIG_EXAMPLES_SENSORA_DEVPATH, O_RDONLY);
	if (fd_sensora < 0)
	{
		printf("fd_sensora: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errno);
	}

	fd_sensorb = open(CONFIG_EXAMPLES_SENSORB_DEVPATH, O_RDONLY);
	if (fd_sensorb < 0)
	{
		printf("fd_sensorb: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errno);
	}

	
	while(ret == FAIL)
	{
		ret = gprs_rst(fd,gprsdata);
		if(cnt++ > 3)
		{
			cnt = 0;
			Gprsfail_buzz_alarm();			
			break;
		}
	}
	if(ret == SUCCESS)
	{
        //changed by liubofei 2017-12-26
		check_lockstate(fd_sensora,fd_sensorb,&Hall_Sensor);
		//end	
		ret = gprs_openlock(fd,gprsdata,adc_dada,sensor,1);
		if(ret == SUCCESS)
		{
			//close by liubofei 2017-12-26 //temporarily remove
			#if 0
			if(gprsdata->download_data.locker == ENABLE)
			{
				//ask open lock
				*locker = ASK_OPENLOCK;
			}
			/*	//close by liubofei 2017-12-26 //temporarily remove
			else if(gprsdata->download_data.locker == DISABLE)
			{
				//ask close lock
				*locker = ASK_CLOSELOCK;
			}
			*/
			#endif
			gprsdata->process_state = SUCCESS;
			
			
		}
		else
		{
			//开锁请求失败，会把锁关闭，逻辑可能不读,暂时去掉
			//ask close lock
			//*locker = ASK_CLOSELOCK;
			gprsdata->process_state = FAIL;
		}
		
		//new add by liubofei 2017-12-26
		if(gprsdata->process_state == SUCCESS)
		{
			int cnt_lock = 0;

			if(gprsdata->download_data.locker == ENABLE)
			{
				//ask open lock
				*locker = ASK_OPENLOCK;
			}


			while(strcmp(sensor->lockstr,"on"))
			{
				check_lockstate(fd_sensora,fd_sensorb,&Hall_Sensor);
				usleep(500*1000);//0.5s
				if(cnt_lock++ > 60)
				{
					cnt_lock = 0;
					break;
				}
			}
			
			ret = gprs_openlock(fd,gprsdata,adc_dada,sensor,2);

		}
		//end
	}
	else
	{
		//开锁请求失败，会把锁关闭，逻辑可能不读,暂时去掉
		//ask close lock
		//*locker = ASK_CLOSELOCK;
		printf("gprs connect fail\n");
	}

	close(fd_sensora);
	close(fd_sensorb);
	
}
/****************************************************************************
 * w315mhz_ask_closelock_process
 * liushuhe
 * 2017.11.27
 ****************************************************************************/
void w315mhz_ask_closelock_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int ret = FAIL;
	int cnt = 0;

	int fd_sensora;
	int fd_sensorb;
	//ask close lock
	*locker = ASK_CLOSELOCK;
	
//changed by liubofei 2017-12-26 (check lockstate --> report data )

	int cnt_lock = 0;

	fd_sensora = open(CONFIG_EXAMPLES_SENSORA_DEVPATH, O_RDONLY);
	if (fd_sensora < 0)
	{
		printf("fd_sensora: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errno);
	}

	fd_sensorb = open(CONFIG_EXAMPLES_SENSORB_DEVPATH, O_RDONLY);
	if (fd_sensorb < 0)
	{
		printf("fd_sensorb: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errno);
	}

	while(strcmp(sensor->lockstr,"off"))
	{
		check_lockstate(fd_sensora,fd_sensorb,&Hall_Sensor);
		usleep(500*1000);//0.5s
		if(cnt_lock++ > 60)
		{
				cnt_lock = 0;
				//close(fd_sensora);
				//close(fd_sensorb);
				break;
		}
	}
	close(fd_sensora);
	close(fd_sensorb);
//end
	while(ret == FAIL)
	{
		ret = gprs_rst(fd,gprsdata);
		if(cnt++ > 3)
		{
			cnt = 0;
			Gprsfail_buzz_alarm();			
			break;
		}
	}
	if(ret == SUCCESS)
	{
		sprintf(sensor->lockstr,"off");
		ret = gprs_closelock(fd,gprsdata,adc_dada,sensor);
		if(ret == SUCCESS)
		{
			gprsdata->process_state = SUCCESS;
		}
		else
		{
			gprsdata->process_state = FAIL;
		}
	}
	else
	{
		printf("gprs connect fail\n");
	}
}

/****************************************************************************
 * w315mhz_ask_openlock_process
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
void bluetooth_openlock_process(int fd,int  *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor)
{
	int ret = FAIL;
	int cnt = 0;
	while(ret == FAIL)
	{
		ret = gprs_rst(fd,gprsdata);
		if(cnt++ > 3)
		{
			cnt = 0;
			Gprsfail_buzz_alarm();			
			break;
		}
	}
	if(ret == SUCCESS)
	{
		ret = gprs_openlock(fd,gprsdata,adc_dada,sensor,1);
		if(ret == SUCCESS)
		{
			if(gprsdata->download_data.locker == ENABLE)
			{
				//ask open lock
				*locker = ASK_OPENLOCK;
			}
			else if(gprsdata->download_data.locker == DISABLE)
			{
				//ask close lock
				*locker = ASK_CLOSELOCK;
			}
			gprsdata->process_state = SUCCESS;
		}
		else
		{
			//ask close lock
			*locker = ASK_CLOSELOCK;
			gprsdata->process_state = FAIL;
		}
	}
	else
	{
		//ask close lock
		*locker = ASK_CLOSELOCK;
		printf("gprs connect fail\n");
	}
}
/****************************************************************************
 * master_monitor
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
int master_monitor(int argc, char *argv[])
{
	int fd_gprs_copy;
	int fd_rtc;
	g_monitor_started = false;
	
	DisLocalTime.system_init = 0;//new add by liubofei 2017-12-29
	
	fd_gprs_copy = open(CONFIG_EXAMPLES_GPRS_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_gprs_copy < 0)
	{
		int errcode = errno;
		printf("gprs: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_GPRS_DEVPATH, errcode);
	}
	
	fd_rtc = open(CONFIG_EXAMPLES_RTC_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_rtc < 0)
	{
		int errcode = errno;
		printf("rtc0: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_RTC_DEVPATH, errcode);
	}
	
	getRtcTime(fd_rtc,&rtctime);
	printf("read rtc:%d%d%d%d%d%d\n",rtctime.tm_year,rtctime.tm_mon,rtctime.tm_mday,
						  rtctime.tm_hour,rtctime.tm_min,rtctime.tm_sec);
	setSystime(&rtctime);
    sleep(5);
	//new add by liubofei 2017-12-26
	first_getvcc(&g_AdcConVar,&g_AdcMutex,DATA_NUM);
	sleep(1);
	//end
	while(1)
	{
		usleep(1000*1000L);                                     //sleep 100ms
		/*************************************************************************/
		//启动一次adc采集
		//changed by liubofei 2017-12-26
		EnAdcSampl(&g_AdcConVar,&g_AdcMutex);
		
		//getVccValue(&g_AdcConVar,&g_AdcMutex);
		
		if(1==SensorDate.sampleisok)
		{
			SensorDate.sampleisok	= 0;
			/*************************************************************************/
			printf("%s: value: %d \n", "sample_tempdata[0]", SensorDate.sample_tempdata[0].am_data);
			printf("%s: value: %d \n", "sample_tempdata[1]", SensorDate.sample_tempdata[1].am_data);
			printf("%s: value: %d \n", "sample_tempdata[2]", SensorDate.sample_tempdata[2].am_data);
			//VCC
			//new add by liubofei 2017-12-26
			if(abs(SensorDate.adcmsg->VCC - SensorDate.adcmsg->VCC_middle) <= 2.0)
			{
				SensorDate.adcmsg->VCCTemp[DATA_NUM-1] = SensorDate.adcmsg->VCC;
			}
			SensorDate.adcmsg->VCC_middle=filter(DATA_NUM);
			
			printf("%s: value: %.2f %.1fv\n", "VCC_middle",SensorDate.adcmsg->VCC_middle,(int)(SensorDate.adcmsg->VCC_middle*10)/10.0);
			printf("%s: value: %.2f m\n", "SensorDate.adcmsg.Water_high", SensorDate.adcmsg->Water_high);
			printf("%s: value: %.2f v\n", "SensorDate.adcmsg.Light", SensorDate.adcmsg->Light);

			//end
			
			/*************************************************************************/
			//紧急上报
			//changed by liubofei 2017-12-26
			//if((SensorDate.adcmsg->VCC <= alarmdata.vcc_mb)||(SensorDate.adcmsg->Water_high >= alarmdata.water ))
			
			printf("SensorDate.adcmsg->VCC_middle =%.1f,SensorDate.adcmsg->Water_high=%.2f\n",SensorDate.adcmsg->VCC_middle,SensorDate.adcmsg->Water_high);
			printf("alarmdata.vcc_mb=%.1f,alarmdata.water=%.2f\n",alarmdata.vcc_mb,alarmdata.water);
			printf("timingA=%2d:%2d,timingB=%2d:%2d\n",alarmdata.timingA_hour,alarmdata.timingA_min,alarmdata.timingB_hour,alarmdata.timingB_min);
			if((SensorDate.adcmsg->VCC_middle <= alarmdata.vcc_mb)||(SensorDate.adcmsg->Water_high >= alarmdata.water ))
			{
				if(!pthread_mutex_trylock(&g_MonitorMutex))
				{
					printf("WarningUpload get g_MonitorMutex Lock OK!\n");
					Msg485Data.type 	=	WARN_UPLOAD;
					pthread_mutex_unlock(&g_MonitorMutex);
				}
				else
				{
					printf("WarningUpload get g_MonitorMutex Lock fail!\n");
				}
			}
		}
		/*************************************************************************/
		//485
		switch(Msg485Data.type)
		{
			//warn upload
			case WARN_UPLOAD:
					if(GprsData.process_state != SUCCESS)
					{
						printf("warn_upload_process  starting................\n");
						warn_upload_process(fd_gprs_copy,&Locker,&GprsData,&adcdata,&Hall_Sensor);
					}
					
					Msg485Data.type = WAIT;
					boardctl(BOARDIOC_GPRS_PWROFF, 0);
				break;
			//timeint upload	
			case TIMEINT_UPLOAD:
					if(GprsData.process_state != SUCCESS)
					{
						printf("timeint_upload_process  starting................\n");
						timeint_upload_process(fd_gprs_copy,&Locker,&GprsData,&adcdata,&Hall_Sensor);
					}
					Msg485Data.type = WAIT;
					boardctl(BOARDIOC_GPRS_PWROFF, 0);
				break;
			//open lock
			case OPEN_LOCK:
					if(GprsData.process_state != SUCCESS)
					{
						printf("w315mhz_ask_openlock_process  starting................\n");
						w315mhz_ask_openlock_process(fd_gprs_copy,&Locker,&GprsData,&adcdata,&Hall_Sensor);
					}
					Msg485Data.type = WAIT;
					boardctl(BOARDIOC_GPRS_PWROFF, 0);
				break;
			//close lock
			case CLOSE_LOCK:
					if(GprsData.process_state != SUCCESS)
					{
						printf("w315mhz_ask_closelock_process  starting................\n");
						w315mhz_ask_closelock_process(fd_gprs_copy,&Locker,&GprsData,&adcdata,&Hall_Sensor);
					}
					Msg485Data.type = WAIT;
					boardctl(BOARDIOC_GPRS_PWROFF, 0);
				break;
		}
		//bluetooth
		switch(bluetooth_msg.type)
		{
			//open lock
			case BT_OPEN_LOCK:
					if(GprsData.process_state != SUCCESS)
					{
						bluetooth_msg.type = WAIT;
						printf("bluetooth_openlock_process  starting................\n");
						bluetooth_openlock_process(fd_gprs_copy,&Locker,&GprsData,&adcdata,&Hall_Sensor);
						Msg485Data.type = POWER_OFF;
					}
					else
					{
						Msg485Data.type = WAIT;
					}
					boardctl(BOARDIOC_GPRS_PWROFF, 0);
				break;
		}
		/*************************************************************************/
		//定时采集 8:00 | 18:00
		if(1 == TimeInt_SampleFlag)
		{
  			pthread_mutex_lock(&g_TimIntMutex);
			sleep(1);
			TimeInt_SampleFlag = 0;	
			pthread_mutex_unlock(&g_TimIntMutex);
			if(0 == pthread_mutex_trylock(&g_MonitorMutex))
			{
				printf("TimeIntUpload get g_MonitorMutex Lock OK!\n");
				Msg485Data.type 	=	TIMEINT_UPLOAD;
				pthread_mutex_unlock(&g_MonitorMutex);
			}
			else
			{
				printf("TimeIntUpload get g_MonitorMutex Lock fail!\n");
			}
		}
		/*************************************************************************/

	}
  printf("master_monitor: Terminating\n");

 return EXIT_FAILURE;

}


