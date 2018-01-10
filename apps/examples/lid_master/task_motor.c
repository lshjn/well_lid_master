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
#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_485.h"
#include "task_motor.h"
#include "task_monitor.h"
#include "task_adc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
int  fd_sensorA;
int  fd_sensorB;
int  fd_light;
int  fd_pwm;
int  buzzpwm_fd;
#if 0
int  fd_lock_open;//new add by liubofei 2017-12-28
int  fd_lock_close;//new add by liubofei 2017-12-28
#endif
int  Locker;

struct	hall_sensor  Hall_Sensor;
struct 	pwm_state_s g_pwmstate;

/****************************************************************************
 * pwm_init
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
void	pwm_init(int fd,struct 	pwm_state_s *pwmstate)
{
	struct pwm_info_s info;
	int ret;
	if (!pwmstate->initialized)
	{
		pwmstate->duty        = CONFIG_EXAMPLES_PWM_DUTYPCT;
		pwmstate->freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
		pwmstate->initialized = true;
	}
	
	info.frequency = pwmstate->freq;
	info.duty  = ((uint32_t)pwmstate->duty << 16) / 100;
	printf("pwm_main: starting output with frequency: %u duty: %08x\n",info.frequency, info.duty);
	//set pwm info
	ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
	if (ret < 0)
	{
		printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}
}
/****************************************************************************
 * buzzon_pwm_init
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void	buzzon_pwm_init(int fd)
{
	struct 	pwm_state_s pwmstate;
	struct pwm_info_s info;
	int ret;
	
	pwmstate.duty        = CONFIG_EXAMPLES_BUZZON_PWM_DUTYPCT;
	pwmstate.freq        = CONFIG_EXAMPLES_BUZZON_PWM_FREQUENCY;
	
	info.frequency = pwmstate.freq;
	info.duty  = ((uint32_t)pwmstate.duty << 16) / 100;
	printf("buzzon_pwm_init: starting output with frequency: %u duty: %08x\n",info.frequency, info.duty);
	//set pwm info
	ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
	if (ret < 0)
	{
		printf("buzzon_pwm_init: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}
}

/****************************************************************************
 * buzzon_pwm_init
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void	buzzalarm_pwm_init(int fd)
{
	struct 	pwm_state_s pwmstate;
	struct pwm_info_s info;
	int ret;
	
	pwmstate.duty        = 10;
	pwmstate.freq        = 4;
	
	info.frequency = pwmstate.freq;
	info.duty  = ((uint32_t)pwmstate.duty << 16) / 100;
	printf("buzzalarm_pwm_init: starting output with frequency: %u duty: %08x\n",info.frequency, info.duty);
	//set pwm info
	ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
	if (ret < 0)
	{
		printf("buzzalarm_pwm_init: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}
}

/****************************************************************************
 * buzzoff_pwm_init
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void	buzzoff_pwm_init(int fd)
{
	struct 	pwm_state_s pwmstate;
	struct pwm_info_s info;
	int ret;
	
	pwmstate.duty        = CONFIG_EXAMPLES_BUZZOFF_PWM_DUTYPCT;
	pwmstate.freq        = CONFIG_EXAMPLES_BUZZOFF_PWM_FREQUENCY;
	
	info.frequency = pwmstate.freq;
	info.duty  = ((uint32_t)pwmstate.duty << 16) / 100;
	printf("buzzoff_pwm_init: starting output with frequency: %u duty: %08x\n",info.frequency, info.duty);
	//set pwm info
	ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
	if (ret < 0)
	{
		printf("buzzoff_pwm_init: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}
}
/****************************************************************************
 * buzz_alarm
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void	buzz_alarm(void)
{
	int fd;
	fd = open(CONFIG_EXAMPLES_BUZZPWM_DEVPATH, O_RDONLY);
	if (fd < 0)
	{
		printf("buzz_fd: open %s failed: %d\n", CONFIG_EXAMPLES_BUZZPWM_DEVPATH, errno);
	}
	buzzalarm_pwm_init(fd);
	ioctl(fd, PWMIOC_START, 0);
	sleep(1);
	ioctl(fd, PWMIOC_STOP, 0);
	close(fd);
}

/****************************************************************************
 * Gprsfail_buzz_alarm
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void	Gprsfail_buzz_alarm(void)
{
	int fd;
	int cnt=0;
	fd = open(CONFIG_EXAMPLES_BUZZPWM_DEVPATH, O_RDONLY);
	if (fd < 0)
	{
		printf("buzz_fd: open %s failed: %d\n", CONFIG_EXAMPLES_BUZZPWM_DEVPATH, errno);
	}	
	do
	{
		buzzon_pwm_init(fd);
		ioctl(fd, PWMIOC_START, 0);
		sleep(1);
		ioctl(fd, PWMIOC_STOP, 0);
		buzzalarm_pwm_init(fd);
		ioctl(fd, PWMIOC_START, 0);
		sleep(2);
		ioctl(fd, PWMIOC_STOP, 0);
	}while(cnt++<=3);
	close(fd);
}


/****************************************************************************
 * pwm_start
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
void	pwm_start(int fd)
{
	int ret;
	
	ret = ioctl(fd, PWMIOC_START, 0);
	if (ret < 0)
	{
		printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
	}
}
/****************************************************************************
 * pwm_stop
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
void	pwm_stop(int fd)
{
	int ret;

	printf("pwm_main: stopping output\n");

	ret = ioctl(fd, PWMIOC_STOP, 0);
	if (ret < 0)
	{
		printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
	}
}


/****************************************************************************
 * check_sensorA_sinal
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
int	check_sensorA_sinal(int fd,struct	hall_sensor *sensor)
{
	int ret;
	
	ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&sensor->sensorA_State));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read value from %s: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errcode);
		close(fd);
		return EXIT_FAILURE;
	}
	
	return 1;
}

/****************************************************************************
 * check_sensorB_sinal
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int	check_sensorB_sinal(int fd,struct	hall_sensor *sensor)
{
	int ret;
	
	ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&sensor->sensorB_State));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read value from %s: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errcode);
		close(fd);
		return EXIT_FAILURE;
	}
	return 1;
}
#if 0
/****************************************************************************
 * check_light_315sinal
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int	check_light_315sinal(int fd,struct	hall_sensor *sensor)
{
	int ret;
	
	ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&sensor->light_State));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read value from %s: %d\n", CONFIG_EXAMPLES_LIGHT315_DEVPATH, errcode);
		close(fd);
		return EXIT_FAILURE;
	}
	return 1;
}
#endif
/****************************************************************************
 * check_lockstate
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
void  check_lockstate(int fd_a,int fd_b,struct	 hall_sensor *sensor)
{
	check_sensorA_sinal(fd_a,sensor);
	check_sensorB_sinal(fd_b,sensor);
	if(sensor->sensorA_State == LOWER)
	{
		sensor->locker_state = OPEN;
		sprintf(sensor->lockstr,"on");
	}
	else if(sensor->sensorB_State == LOWER)
	{
		sensor->locker_state = CLOSE;
		sprintf(sensor->lockstr,"off");
	}
	else if((sensor->sensorA_State == HIGH)&&(sensor->sensorB_State == HIGH))	
	{
		sensor->locker_state = ALARM;
		sprintf(sensor->lockstr,"alarm");
	}
}

/****************************************************************************
 * ask_openlock
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
void	ask_openlock(int fd_a,int fd_b,int fd_Pwm,struct  hall_sensor *sensor)
{	
	bool  motor_star = false;
	
	check_lockstate(fd_a,fd_b,sensor);
	//lock no open
	while((sensor->locker_state == CLOSE)||(sensor->locker_state == ALARM))
	{
		if(motor_star != true)
		{
			motor_star = true;
			//电机正转
			printf("start pwm ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
			boardctl(BOARDIOC_MOTOR_EN_ENABLE, 0);
			boardctl(BOARDIOC_MOTOR_DIR_OPEN, 0);
			pwm_start(fd_Pwm);			
		}
		usleep(1*1000);
		check_lockstate(fd_a,fd_b,sensor);
	}
	//电机断电
	boardctl(BOARDIOC_MOTOR_EN_DISABLE, 0);
	pwm_stop(fd_Pwm);
}
/****************************************************************************
 * ask_closelock
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
void	ask_closelock(int fd_a,int fd_b,int fd_Pwm,struct  hall_sensor *sensor)
{
	bool  motor_star = false;
	
	check_lockstate(fd_a,fd_b,sensor);
	//lock no close
	while((sensor->locker_state == OPEN)||(sensor->locker_state == ALARM))
	{
		if(motor_star != true)
		{
			motor_star = true;
			//电机反转
			printf("start pwm -----------------------------------------------------------------------\n");
			boardctl(BOARDIOC_MOTOR_EN_ENABLE, 0);
			boardctl(BOARDIOC_MOTOR_DIR_CLOSE, 0);
			pwm_start(fd_Pwm);			
		}
		usleep(1*1000);
		check_lockstate(fd_a,fd_b,sensor);
	}
	//电机断电
	boardctl(BOARDIOC_MOTOR_EN_DISABLE, 0);
	pwm_stop(fd_Pwm);
}

/****************************************************************************
 * master_485
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_motor(int argc, char *argv[])
{
	
	fd_sensorA = open(CONFIG_EXAMPLES_SENSORA_DEVPATH, O_RDONLY);
	if (fd_sensorA < 0)
	{
		printf("fd_sensorA: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORA_DEVPATH, errno);
		goto errout;
	}

	fd_sensorB = open(CONFIG_EXAMPLES_SENSORB_DEVPATH, O_RDONLY);
	if (fd_sensorB < 0)
	{
		printf("fd_sensorB: open %s failed: %d\n", CONFIG_EXAMPLES_SENSORB_DEVPATH, errno);
		goto errout;
	}
	//close by liubofei 2017-12-26
	/*
	fd_light = open(CONFIG_EXAMPLES_LIGHT315_DEVPATH, O_RDONLY);
	if (fd_light < 0)
	{
		printf("fd_light: open %s failed: %d\n", CONFIG_EXAMPLES_LIGHT315_DEVPATH, errno);
		goto errout;
	}
   */
	fd_pwm = open(CONFIG_EXAMPLES_PWM_DEVPATH, O_RDONLY);
	if (fd_pwm < 0)
	{
		printf("pwm_main: open %s failed: %d\n", CONFIG_EXAMPLES_PWM_DEVPATH, errno);
		goto errout;
	}
	buzzpwm_fd = open(CONFIG_EXAMPLES_BUZZPWM_DEVPATH, O_RDONLY);
	if (buzzpwm_fd < 0)
	{
		printf("buzz_fd: open %s failed: %d\n", CONFIG_EXAMPLES_BUZZPWM_DEVPATH, errno);
		goto errout;
	}


	boardctl(BOARDIOC_PWM_GPIOINIT, 0);
	
	pwm_init(fd_pwm,&g_pwmstate);

	check_lockstate(fd_sensorA,fd_sensorB,&Hall_Sensor);


/*
	sleep(5);
	//电机反转
	printf("start pwm ---\n");
	boardctl(BOARDIOC_MOTOR_DIR_CLOSE, 0);
	pwm_start(fd_pwm);	

	sleep(30);
	boardctl(BOARDIOC_MOTOR_EN_ENABLE, 0);
	//电机正转
	printf("start pwm +++\n");
	boardctl(BOARDIOC_MOTOR_DIR_OPEN, 0);
	pwm_start(fd_pwm);	
	sleep(30);
	//电机反转
	printf("start pwm ---\n");
	boardctl(BOARDIOC_MOTOR_DIR_CLOSE, 0);
	pwm_start(fd_pwm);	
	
	//电机断电
	sleep(30);
	boardctl(BOARDIOC_MOTOR_EN_DISABLE, 0);
	pwm_stop(fd_pwm);
*/


	while(1)
	{
		usleep(50*1000);
		check_sensorA_sinal(fd_sensorA,&Hall_Sensor);
		check_sensorB_sinal(fd_sensorB,&Hall_Sensor);
		/*
		check_light_315sinal(fd_light,&Hall_Sensor);
		
		if(Hall_Sensor.light_State)
		{
			boardctl(BOARDIOC_LIGHT_CTL_OPEN, 0);
		}
		else
		{
			boardctl(BOARDIOC_LIGHT_CTL_CLOSE, 0);
		}
		*/
		switch(Locker)
		{
			case ASK_OPENLOCK:
					buzzon_pwm_init(buzzpwm_fd);
					ioctl(buzzpwm_fd, PWMIOC_START, 0);
					sleep(3);
					ioctl(buzzpwm_fd, PWMIOC_STOP, 0);
					//open
					ask_openlock(fd_sensorA,fd_sensorB,fd_pwm,&Hall_Sensor);
					Locker	= LOCK_OK;
					
					buzzoff_pwm_init(buzzpwm_fd);
					ioctl(buzzpwm_fd, PWMIOC_START, 0);
					sleep(5);
					ioctl(buzzpwm_fd, PWMIOC_STOP, 0);
				break;
			case ASK_CLOSELOCK:
					sleep(5);
					if((Hall_Sensor.sensorZD_State !=1)&&(SensorDate.adcmsg->Light >= 2500))
					{
						buzzon_pwm_init(buzzpwm_fd);
						ioctl(buzzpwm_fd, PWMIOC_START, 0);
						sleep(3);
						ioctl(buzzpwm_fd, PWMIOC_STOP, 0);
						
						//close
						ask_closelock(fd_sensorA,fd_sensorB,fd_pwm,&Hall_Sensor);
						Locker	= LOCK_OK;
						
						buzzoff_pwm_init(buzzpwm_fd);
						ioctl(buzzpwm_fd, PWMIOC_START, 0);
						sleep(5);
						ioctl(buzzpwm_fd, PWMIOC_STOP, 0);		
					}
				break;
		}

		
	}
 errout:
  printf("master_motor: Terminating\n");

  return EXIT_FAILURE;
	
}

