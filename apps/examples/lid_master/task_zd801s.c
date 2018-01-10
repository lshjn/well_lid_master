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

#include "task_485.h"
#include "task_zd801s.h"
#include "task_monitor.h"
#include "task_motor.h"
#include "task_wakeup315.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
static bool  g_zd801s_started;
static	int  fd_zd801s;
int  zd801s_signal_flag = 0;

int		zd801s_cnt=0;

/******************************************************************************************************************************
函数名称：void	zd801s_alarm(struct msg485 *msg485)
功    能：震动报警
输入参数：struct msg485
输入参数：无
编写时间：2017.10.09
编 写 人：liushuhe
*******************************************************************************************************************************/
void	zd801s_alarm(struct msg485 *msg485,struct hall_sensor *Sensor)
{
	
	if(!pthread_mutex_trylock(&g_485Mutex))
	{
		
		printf("ZD801S  alarm............................................................\n");
		msg485->type 	=	ZD801S_ALARM;
		Sensor->sensorZD_State	= 1;
		zd801s_cnt =0;
		pthread_mutex_unlock(&g_485Mutex);
	}
	else
	{
		printf("Wakeup get g_485Mutex Lock fail!\n");
	}
}


/****************************************************************************
 * zd801s_action
 * liushuhe
 * 2017.10.09
 ****************************************************************************/
void zd801s_action(int signo,siginfo_t *siginfo, void *arg)
{
	static int cnt; 
	if (signo == SIGUSR1)
	{
		printf("%2d SIGUSR1 received\n",cnt++);
		zd801s_signal_flag = 1;
	}
}


/****************************************************************************
 * alarm_zd801s
 * liushuhe
 * 2017.10.09
 ****************************************************************************/
int alarm_zd801s(int argc, char *argv[])
{
	enum gpio_pintype_e pintype;
	struct sigaction act;
	struct sigaction oldact;
	int ret;
	int status;

	
	g_zd801s_started = true;

	fd_zd801s = open(CONFIG_EXAMPLES_ZD801S_DEVPATH, O_RDONLY);
	if (fd_zd801s < 0)
	{
		printf("fd_zd801s: open %s failed: %d\n", CONFIG_EXAMPLES_ZD801S_DEVPATH, errno);
		goto errout;
	}
	ret = ioctl(fd_zd801s, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", CONFIG_EXAMPLES_ZD801S_DEVPATH, errcode);
		close(fd_zd801s);
		return EXIT_FAILURE;
	}

	//signal
	memset(&act, 0, sizeof(struct sigaction));
	act.sa_sigaction = zd801s_action;
	act.sa_flags     = SA_SIGINFO;

	(void)sigemptyset(&act.sa_mask);

	status = sigaction(SIGUSR1, &act, &oldact);
	if (status != 0)
	{
		fprintf(stderr, "Failed to install SIGUSR1 handler, errno=%d\n",errno);
		exit(2);
	}
	
	// Set up to receive signal 
	if(pintype == GPIO_INTERRUPT_PIN)
	{	
		ret = ioctl(fd_zd801s, GPIOC_REGISTER, (unsigned long)SIGUSR1);
		if (ret < 0)
		{
			int errcode = errno;
			fprintf(stderr, "ERROR: Failed to setup for signal from %s: %d\n", CONFIG_EXAMPLES_ZD801S_DEVPATH, errcode);
			close(fd_zd801s);
			return EXIT_FAILURE;
		}
	}
	while(1)
	{
		usleep(200*1000);
		if(zd801s_signal_flag == 1)
		{
			zd801s_signal_flag = 0;
			buzz_alarm();
			if(!pthread_mutex_trylock(&g_MonitorMutex))
			{
				printf("Wakeup get g_MonitorMutex Lock ok!\n");
				zd801s_alarm(&Msg485Data,&Hall_Sensor);
				pthread_mutex_unlock(&g_MonitorMutex);
			}
			else
			{
				printf("Wakeup get g_MonitorMutex Lock fail!\n");
			}
		}
		if(Hall_Sensor.sensorZD_State	== 1)
		{
			if(zd801s_cnt++>5*5)
			{
				zd801s_cnt = 0;
				Hall_Sensor.sensorZD_State = 0;
			}
				
		}
	}
 errout:
  g_zd801s_started = false;

  printf("alarm_zd801s: Terminating\n");

  return EXIT_FAILURE;
}






