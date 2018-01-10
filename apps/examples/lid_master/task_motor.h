#ifndef _TASK_MOTOR_H
#define _TASK_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 


//hall gpio value
#define		LOWER	0
#define		HIGH	1

//lock  state
#define		CLOSE	0
#define		OPEN	1
#define		ALARM	2
//lock  cmd

#define    ASK_OPENLOCK	1
#define    ASK_CLOSELOCK	2
#define    LOCK_OK	    	3

#ifndef CONFIG_EXAMPLES_PWM_DUTYPCT
#  define CONFIG_EXAMPLES_PWM_DUTYPCT 50
#endif

#ifndef CONFIG_EXAMPLES_PWM_FREQUENCY
#  define CONFIG_EXAMPLES_PWM_FREQUENCY 100
#endif




/****************************************************************************
 * Private Data
 ****************************************************************************/
struct	hall_sensor
{
	bool	locker_state;
	char    lockstr[10];
	
	int  	sensorA_State;
	int  	sensorB_State;
	int  	sensorZD_State;
	int  	light_State;
	int 		lock_open_State;//new add by liubofei 2017-12-28
	int 		lock_close_State;//new add by liubofei 2017-12-28
};

struct pwm_state_s
{
  bool      initialized;
  uint8_t   duty;
  uint32_t  freq;
  uint32_t  count;
};

extern struct 	pwm_state_s g_pwmstate;
extern struct	hall_sensor  Hall_Sensor;
extern int		Locker;
extern int  	fd_sensorA;
extern int  	fd_sensorB;

extern int  	fd_pwm;
extern int  	buzzpwm_fd;

/****************************************************************************
 * Private function
 ****************************************************************************/
int 	master_motor(int argc, char *argv[]);
int		check_sensorA_sinal(int fd,struct	hall_sensor *sensor);
int		check_sensorB_sinal(int fd,struct	hall_sensor *sensor);
void  	check_lockstate(int fd_a,int fd_b,struct	 hall_sensor *sensor);
void	ask_openlock(int fd_a,int fd_b,int fd_Pwm,struct  hall_sensor *sensor);
void	ask_closelock(int fd_a,int fd_b,int fd_Pwm,struct  hall_sensor *sensor);

void	buzzon_pwm_init(int fd);
void	buzzoff_pwm_init(int fd);
void	buzz_alarm(void);
void	buzzalarm_pwm_init(int fd);
int		check_light_315sinal(int fd,struct	hall_sensor *sensor);
void	Gprsfail_buzz_alarm(void);



#ifdef __cplusplus
}
#endif

#endif 
