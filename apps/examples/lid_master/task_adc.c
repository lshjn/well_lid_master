#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_adc.h"
#include "task_monitor.h"
#include "task_485.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
 
pthread_mutex_t g_AdcMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_AdcConVar	= PTHREAD_COND_INITIALIZER;

struct sensor_msg	SensorDate;
struct adc_msg		adcdata;
struct i2c_msg		i2cdata;

static bool  g_adc_started;

/****************************************************************************/
/******************************************************************************************************************************
�������ƣ�void	EnAdcSampl(pthread_cond_t *cond,pthread_mutex_t *mutex)
��    �ܣ�ʹ��adc����
���������������������
�������������״̬
��дʱ�䣺2017.09.29
�� д �ˣ�liushuhe
*******************************************************************************************************************************/
void	EnAdcSampl(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	//����һ��adc�ɼ�
	pthread_mutex_lock(mutex);
	SensorDate.startadc = true;
	pthread_cond_signal(cond);
	pthread_mutex_unlock(mutex);
}


/******************************************************************************************************************************
�������ƣ�int	StartADCsampl(int fd)
��    �ܣ�����һ��adc����
���������adc�ļ�������
�������������״̬
��дʱ�䣺2017.09.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/

int	StartAdcSampl(int fd)
{
	int	ret;
#ifdef CONFIG_EXAMPLES_ADC_SWTRIG

	//����adcת��
	ret = ioctl(fd, ANIOC_TRIGGER, 0);
	if (ret < 0)
	{
		int errcode = errno;
		printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
	}
#endif
	return ret;
}

/******************************************************************************************************************************
�������ƣ�int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
��    �ܣ���ȡ����ֵ
���������T_adcsample�ṹָ��
�������������״̬
��дʱ�䣺2017.09.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/
int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
{
	struct adc_msg_s sample[CONFIG_EXAMPLES_ADC_GROUPSIZE];

	size_t readsize;
	ssize_t nbytes;
	int errval = 0;
	int adc_num = 0;
	
	//��ȡ�������� 
	readsize = CONFIG_EXAMPLES_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
	nbytes = read(fd, sample, readsize);
	//�����Ƿ��쳣
	if (nbytes < 0)
	{
		errval = errno;
		if (errval != EINTR)
		{
			printf("adc_main: read %s failed: %d\n",CONFIG_EXAMPLES_ADC_DEVPATH, errval);
		}
		printf("adc_main: Interrupted read...\n");
	}
	else if (nbytes == 0)
	{
		printf("adc_main: No data read, Ignoring\n");
	}
	//��ȡ���Ĳ�������
	else
	{
		int nsamples = nbytes / sizeof(struct adc_msg_s);
		if (nsamples * sizeof(struct adc_msg_s) != nbytes)
		{
			printf("adc_main: read size=%ld is not a multiple of sample size=%d, Ignoring\n",(long)nbytes, sizeof(struct adc_msg_s));
		}
		else
		{
			for (adc_num = 0; adc_num < nsamples; adc_num++)
			{
				Sensor_data->sample_tempdata[adc_num].am_data	=	sample[adc_num].am_data;
			}
		}
	}

	return	(int)nbytes;
	
}
/******************************************************************************************************************************
�������ƣ�int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
��    �ܣ���ȡ����ֵ
���������T_adcsample�ṹָ��
�������������״̬
��дʱ�䣺2017.09.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/
void CalcSampleData(struct sensor_msg *Sensor_data)
{
	//2534/2.042 = adc/vcc1, 12/2.042 = VCC/vcc1
	float vcc_temp = (Sensor_data->sample_tempdata[0].am_data*2.042)/2534;
	vcc_temp	  = (vcc_temp*12.18)/2.042;
	//vcc_temp	  = vcc_temp*10;
 	Sensor_data->adcmsg->VCC		=	vcc_temp;	

	//0.496/611 = vcc/adc,0.496/4ma = vcc/y ma, 20 ma/5m = (y-4)/z m
	if(Sensor_data->sample_tempdata[1].am_data != 0)
	{
		float Water_temp = (Sensor_data->sample_tempdata[1].am_data*0.495)/611;
		Water_temp = (Water_temp*4)/0.496;
		Water_temp = ((Water_temp-4)*5)/20;
		//Water_temp = Water_temp*10;
		Sensor_data->adcmsg->Water_high =	Water_temp;
		if(Sensor_data->adcmsg->Water_high < 0)
		{
			Sensor_data->adcmsg->Water_high = 0;
		}
	}
	else
	{
		Sensor_data->adcmsg->Water_high =	0;
	}

    Sensor_data->adcmsg->Light = Sensor_data->sample_tempdata[2].am_data;
	
	//ok
	Sensor_data->sampleisok = 1;

}

void  getadcdata(struct sensor_msg *Sensor_data)
{
	int  ret;
	int  fd_adc;
	
	fd_adc = open(CONFIG_EXAMPLES_ADC_DEVPATH, O_RDONLY);
	if (fd_adc < 0)
	{
		printf("slave_adc: open %s failed: %d\n", CONFIG_EXAMPLES_ADC_DEVPATH, errno);
	}
	
	ret = StartAdcSampl(fd_adc);
	//successful
	if(ret == 0)
	{
		ret = ReadAdcData(fd_adc,Sensor_data);
		if(ret)
		{
			CalcSampleData(Sensor_data);
			//test
			//SensorDate.i2cmsg->humidity    = 7.4;
			//SensorDate.i2cmsg->tempretrue  = 15.6;
		}
	}

	close(fd_adc);
}

/****************************************************************************
 * slave_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_adc(int argc, char *argv[])
{
	g_adc_started = true;
	
	SensorDate.adcmsg	= &adcdata;
	SensorDate.i2cmsg	= &i2cdata;
	
	while(1)
	{
		pthread_mutex_lock(&g_AdcMutex);
		while(SensorDate.startadc != true)
		{
			pthread_cond_wait(&g_AdcConVar, &g_AdcMutex);
		}
		
		boardctl(BOARDIOC_LSENSOR_PWRON, 0);
		getadcdata(&SensorDate);
		
		boardctl(BOARDIOC_LSENSOR_PWROFF, 0);
		
		SensorDate.startadc = false;
		pthread_mutex_unlock(&g_AdcMutex);

	}
/*
errout_with_dev:
  close(fd_adc);
 errout:
 */
  g_adc_started = false;

  printf("adc: Terminating\n");

  return EXIT_FAILURE;
}



