#ifndef _TASK_MONITOR_H
#define _TASK_MONITOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <nuttx/timers/rtc.h>

#include "task_485.h"
#include "task_gprs.h"
#include "task_bluetooth.h"
#include "task_motor.h"
#include "task_monitor.h"


/****************************************************************************
 * Private Data
 ****************************************************************************/
#define	CONFIG_EXAMPLES_RTC_DEVPATH	"/dev/rtc0"

extern		pthread_mutex_t g_TimIntMutex;
extern		pthread_mutex_t g_MonitorMutex;
extern 		struct  	TimeStruct 		DisLocalTime;
extern		struct		tm 				*tmp;
extern		time_t		timelocal;
extern		struct 		rtc_time	rtctime;

extern		char		TimeInt_SampleFlag;

/****************************************************************************
 * Private struct
 ****************************************************************************/
struct  TimeStruct
{
    unsigned int   Year;          //年Rtc
    unsigned char  Month;         //月
    unsigned char  Day;           //日
    unsigned char  Hour;          //时
    unsigned char  Minute;        //分
    unsigned char  Second;        //秒
    unsigned long  NTPSecond;
	unsigned long  NTPSecond_check;
	unsigned long  NTPSecond_tmp;
	unsigned int   system_init;
	
};
//new add by liubofei 2017-12-29
struct  TimeStruct_check
{
    unsigned int   Year;          //年Rtc
    unsigned char  Month;         //月
    unsigned char  Day;           //日
    unsigned char  Hour;          //时
    unsigned char  Minute;        //分
    unsigned char  Second;        //秒
    unsigned long  NTPSecond;
	unsigned long  NTPSecond_check;
	unsigned long  NTPSecond_tmp;
	unsigned int   system_init;
	
};
/****************************************************************************
 * Private function
 ****************************************************************************/
int   master_monitor(int argc, char *argv[]);
void  warn_upload_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adcdata,struct hall_sensor *sensor);
void  timeint_upload_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adcdata,struct hall_sensor *sensor);
void  w315mhz_ask_openlock_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adcdata,struct hall_sensor *sensor);
void  bluetooth_openlock_process(int fd,int  *locker,struct gprs_data	*gprsdata,struct adc_msg *adcdata,struct hall_sensor *sensor);
void w315mhz_ask_closelock_process(int fd,int *locker,struct gprs_data	*gprsdata,struct adc_msg *adc_dada,struct hall_sensor *sensor);

void  getSystime(void);
int   setSystime(struct rtc_time *rtc);
int   setRtcTime(int fd,struct rtc_time *rtc);
int   getRtcTime(int fd,struct rtc_time *rtc);
int	  CheckTimeInt(void);



#ifdef __cplusplus
}
#endif

#endif 
