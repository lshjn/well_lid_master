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
#include "task_monitor.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
static bool  g_close315_started;
static	int  fd_close;
int  close_signal_flag = 0;


/******************************************************************************************************************************
函数名称：void	Wakeup_closeLock(struct msg485 *msg485)
功    能：无线唤醒，请求关锁
输入参数：struct msg485
输入参数：无
编写时间：2017.10.09
编 写 人：liushuhe
*******************************************************************************************************************************/
void	Wakeup_closeLock(struct msg485 *msg485)
{
	
	if(!pthread_mutex_trylock(&g_485Mutex))
	{
		printf("close Lock......................................................\n");
		msg485->type 	=	CLOSE_LOCK;
		pthread_mutex_unlock(&g_485Mutex);
	}
	else
	{
		printf("close315 get g_485Mutex Lock fail!\n");
	}
}


/****************************************************************************
 * close315_action 无线遥控唤醒，请求开锁
 * liushuhe
 * 2017.10.09
 ****************************************************************************/
void close315_action(int signo,siginfo_t *siginfo, void *arg)
{
	static int cnt; 
	if (signo == SIGUSR1)
	{
		printf("%2d SIGUSR1 received\n",cnt++);
		close_signal_flag = 1;
	}
}

/***************************************************************************
* check_close315_sinal 检测电平状态
* liubofei
* 2017-12-28
***************************************************************************/
int check_close315_sinal(int fd,struct	hall_sensor *sensor)
{
	int ret = -1;
	ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&sensor->lock_close_State));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read value from %s: %d\n", CONFIG_EXAMPLES_CLOSE315_DEVPATH, errcode);
		close(fd);
		return EXIT_FAILURE;
	}
	return 1;
}


/****************************************************************************
 * closelock_315
 * liushuhe
 * 2017.10.09
 ****************************************************************************/
int closelock_315(int argc, char *argv[])
{
	enum gpio_pintype_e pintype;
	struct sigaction act;
	struct sigaction oldact;
	int ret;
	int status;
	int cnt = 0;
	g_close315_started = true;

	fd_close = open(CONFIG_EXAMPLES_CLOSE315_DEVPATH, O_RDONLY);
	if (fd_close < 0)
	{
		printf("closelock_315: open %s failed: %d\n", CONFIG_EXAMPLES_CLOSE315_DEVPATH, errno);
		goto errout;
	}
	//close by liubofei 2017-12-26
	#if 0
	ret = ioctl(fd_close, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", CONFIG_EXAMPLES_CLOSE315_DEVPATH, errcode);
		close(fd_close);
		return EXIT_FAILURE;
	}

	//signal
	memset(&act, 0, sizeof(struct sigaction));
	act.sa_sigaction = close315_action;
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
		ret = ioctl(fd_close, GPIOC_REGISTER, (unsigned long)SIGUSR1);
		if (ret < 0)
		{
			int errcode = errno;
			fprintf(stderr, "ERROR: Failed to setup for signal from %s: %d\n", CONFIG_EXAMPLES_CLOSE315_DEVPATH, errcode);
			close(fd_close);
			return EXIT_FAILURE;
		}
	}
	#endif
	while(1)
	{
		usleep(20*1000);

	//new add by liubofei 2017-12-26

		check_close315_sinal(fd_close,&Hall_Sensor);

		if(Hall_Sensor.lock_close_State)
		{
			if(cnt++ >5)
			{
				close_signal_flag = 1;
				cnt = 0;
				printf("CLOSE LOCK\nCLOSE LOCK\nCLOSE LOCK\n");
			}
		}
		else
		{
			cnt = 0;
		}	

	//end
		
		if(close_signal_flag == 1)
		{
			close_signal_flag = 0;
			buzz_alarm();
			if(!pthread_mutex_trylock(&g_MonitorMutex))
			{
				printf("close315 get g_MonitorMutex Lock ok!\n");
				Wakeup_closeLock(&Msg485Data);
				pthread_mutex_unlock(&g_MonitorMutex);
			}
			else
			{
				printf("close315 get g_MonitorMutex Lock fail!\n");
			}
			
		}
	}
 errout:
  g_close315_started = false;

  printf("close315: Terminating\n");

  return EXIT_FAILURE;
}






