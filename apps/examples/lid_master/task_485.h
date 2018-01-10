#ifndef _TASK_485_H
#define _TASK_485_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <nuttx/analog/adc.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
extern pthread_mutex_t g_485Mutex;



 //msg_type
#define		WAIT				0
#define		HANDS_ASK			1
#define		HANDS_ACK			2
#define		WARN_UPLOAD		3
#define		TIMEINT_UPLOAD		4
#define		OPEN_LOCK			5
#define		POWER_OFF			6
#define		ZD801S_ALARM		7
#define		CLOSE_LOCK			8

//ack
#define		NOACK				0
#define		ACK					1

#define 	DATA_NUM			11//ji shu

/****************************************************************************
 * Private struct
 ****************************************************************************/
struct msg485
{
	char type;				//1:hands	2:warning_upload		3:TimeInt_upload		4:open_lock	5:power_down
	char msg_state;			//0:noack	1:ack
	char msgbuf[200];
	char msglen;
};

struct adc_msg
{
	float	 tempretrue;
	float	 humidity;
	float	 VCCTemp[11];//new add by liubofei 2017-12-26
	float	 VCC_middle;
	float	 VCC;
	float	 O2;
	float	 NH3;
	float	 H2S;
	float	 CO;
	float	 Water_high;
	float	 Light;
};

struct i2c_msg
{
	int	 tempretrue;
	int	 humidity;
};


struct sensor_msg {
	bool	startadc;
	char	sampleisok;
	struct adc_msg_s	sample_tempdata[3];
	struct adc_msg		*adcmsg;
	struct i2c_msg		*i2cmsg;
};

/****************************************************************************
Private struct
****************************************************************************/

extern  struct msg485		Msg485Data;


/****************************************************************************
 * Private function
 ****************************************************************************/

int		WaitHandOK(struct msg485 *msg485);
void	Hand_485Msg(int fd,struct msg485 *msg485,struct adc_msg *adcdada);

int 	master_485(int argc, char *argv[]);

float  getVccValue(pthread_cond_t *cond,pthread_mutex_t *mutex);
float filter(int N);
int first_getvcc(pthread_cond_t *cond,pthread_mutex_t *mutex,int N);



#ifdef __cplusplus
}
#endif

#endif 
