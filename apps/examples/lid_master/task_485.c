#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <signal.h>
#include <pthread.h>

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_485.h"
#include "task_gprs.h"
#include "task_adc.h"

#include "task_monitor.h"
/****************************************************************************
 * Private Data
 ****************************************************************************/
pthread_mutex_t g_485Mutex		= PTHREAD_MUTEX_INITIALIZER;

struct		msg485		Msg485Data;

static		bool		g_485_started;
static		int			fd_485;

////new add by liubofei 2017-12-26

/******************************************************************************************************************************
函数名称：float  getVccValue(pthread_cond_t *cond,pthread_mutex_t *mutex)
功    能：VCC 数据采集
编写时间：2017.12.26
编 写 人：
*******************************************************************************************************************************/
float  getVccValue(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	
	EnAdcSampl(cond,mutex);
	while(1!=SensorDate.sampleisok)
	{
		usleep(20*1000);
	}
    SensorDate.sampleisok = 0;
	
	return SensorDate.adcmsg->VCC;
}

/******************************************************************************************************************************
函数名称：char filter(int N)
功    能：冒泡排序
编写时间：2017.12.26
编 写 人：
*******************************************************************************************************************************/
float filter(int N)
{
   int i,j;

   float temp; 

   for (j=0;j<N-1;j++)  
   {  
      for (i=0;i<N-1-j;i++)  
      {  
         if ( SensorDate.adcmsg->VCCTemp[i] > SensorDate.adcmsg->VCCTemp[i+1] )  
         {  
             temp = SensorDate.adcmsg->VCCTemp[i];  
             SensorDate.adcmsg->VCCTemp[i] = SensorDate.adcmsg->VCCTemp[i+1];   
             SensorDate.adcmsg->VCCTemp[i+1] = temp;  
         }  
      }  
   }  
   
   SensorDate.adcmsg->VCC_middle = SensorDate.adcmsg->VCCTemp[(N-1)/2];
   return   SensorDate.adcmsg->VCC_middle;

}

/******************************************************************************************************************************
函数名称：int first_getvcc(pthread_cond_t *cond,pthread_mutex_t *mutex,int N)
功    能：第一次开机获取 N 个 VCC 数据，返回一个中间值
编写时间：2017.12.26
编 写 人：
*******************************************************************************************************************************/
int first_getvcc(pthread_cond_t *cond,pthread_mutex_t *mutex,int N)
{
	int i=0;
	
   for ( i=0;i<N;i++)  
   {  
	  SensorDate.adcmsg->VCCTemp[i] = getVccValue(cond,mutex); 
   }  
	for(i=0;i<DATA_NUM;i++)
	{
		printf("[%d]=%.2f  ",i,SensorDate.adcmsg->VCCTemp[i]);
	}
	printf("\n");
   SensorDate.adcmsg->VCC_middle=filter(N);
	for(i=0;i<DATA_NUM;i++)
	{
		printf("[%d]=%.2f  ",i,SensorDate.adcmsg->VCCTemp[i]);
	}
	printf("\n");
   return 0;
}

////end

/******************************************************************************************************************************
函数名称：int	WaitHandOK(struct msg485 *msg485)
功    能：等待握手成功
输入参数：无
输入参数：
编写时间：2017.09.29
编 写 人：liushuhe
*******************************************************************************************************************************/
int	WaitHandOK(struct msg485 *msg485)
{
	msg485->msg_state = NOACK;
	while(msg485->msg_state !=ACK)
	{
		sleep(1);
		msg485->type 	=	HANDS_ASK;
		sprintf(msg485->msgbuf,"$HAND,ASK,EOF\n");	
		msg485->msglen 	=	strlen(msg485->msgbuf);
	}
	return 1;
}
/******************************************************************************************************************************
函数名称：void	Hand_485Msg(int fd,char *msg485->msgbuf,struct msg485 *msg485)
功    能：处理485消息
输入参数：int fd,char *msg485->msgbuf,struct msg485
输入参数：无
编写时间：2017.09.29
编 写 人：liushuhe
*******************************************************************************************************************************/
void	Hand_485Msg(int fd,struct msg485 *msg485,struct adc_msg *adcdada)
{
    char   *pcTempBuf[30];
    char   *pChar = ",";
	int	   cCharNum = 0;
	int  	iBytes = 0;

	printf("rcv:%s\n",msg485->msgbuf);
	/********************************************************************************/
	//power off
	if((Locker == LOCK_OK)||(Msg485Data.type == POWER_OFF))
	{
		printf("send msg: power off.....!\n");
		sprintf(Msg485Data.msgbuf,"$POWER_OFF,ASK,EOF\n");		
		Msg485Data.msglen 	=	strlen(Msg485Data.msgbuf);
		iBytes = write(fd_485,Msg485Data.msgbuf,Msg485Data.msglen);
		if(iBytes == -1)
		{
			printf("Error:write  Data to slave_board\n");
		}
	}
	/********************************************************************************/
	
	//hand
	if(strncmp("$HAND",msg485->msgbuf,strlen("$HAND")) == 0)
	{
		if(strstr(msg485->msgbuf,"$HAND,ASK,EOF"))
		{
			printf("rcv hand ask msg\n");
			sprintf(msg485->msgbuf,"$HAND,ACK,EOF\n");
			msg485->msglen 	=	strlen(msg485->msgbuf);
			iBytes = write(fd,msg485->msgbuf,msg485->msglen);
			if(iBytes == -1)
			{
				printf("Error:write  Data to slave_board\n");
			}
		}
	}
	//warn upload
	else if(strncmp("$WARN_UPLOAD",msg485->msgbuf,strlen("$WARN_UPLOAD")) == 0)
	{	
		cCharNum = 0;
		memset(pcTempBuf, 0, sizeof(pcTempBuf));
		pcTempBuf[0] = strtok((char*)msg485->msgbuf, pChar);
		while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
		{
		}
		//get data
		adcdada->VCC 			= atoi(pcTempBuf[2]);
		adcdada->tempretrue 	= atoi(pcTempBuf[4]);
		adcdada->humidity 		= atoi(pcTempBuf[6]);
		adcdada->Water_high		= atoi(pcTempBuf[8]);
		msg485->type			= WARN_UPLOAD;
		printf("rcv warn upload msg\n");
		if(GprsData.process_state == SUCCESS)
		{
			printf("server:warn upload ok......!!!\n");
			//send ack to slave
			sprintf(msg485->msgbuf,"$WARN_UPLOAD,ACK,EOF\n");		
			msg485->msglen 	=	strlen(msg485->msgbuf);
			iBytes = write(fd,msg485->msgbuf,msg485->msglen);
			if(iBytes == -1)
			{
				printf("Error:write  Data to slave_board\n");
			}
		}
	}
	//timeint upload
	else if(strncmp("$TIMEINT_UPLOAD",msg485->msgbuf,strlen("$TIMEINT_UPLOAD")) == 0)
	{
		cCharNum = 0;
		memset(pcTempBuf, 0, sizeof(pcTempBuf));
		pcTempBuf[0] = strtok((char*)msg485->msgbuf, pChar);
		while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
		{
		}
		//get data
		adcdada->VCC 		= atoi(pcTempBuf[2]);
		adcdada->tempretrue	= atoi(pcTempBuf[4]);
		adcdada->humidity 	= atoi(pcTempBuf[6]);
		adcdada->O2			= atoi(pcTempBuf[8]);
		adcdada->NH3		= atoi(pcTempBuf[10]);
		adcdada->H2S		= atoi(pcTempBuf[12]);
		adcdada->CO			= atoi(pcTempBuf[14]);
		adcdada->Water_high	= atoi(pcTempBuf[16]);
		msg485->type		= TIMEINT_UPLOAD;
		printf("rcv timeint upload msg\n");
		if(GprsData.process_state == SUCCESS)
		{
			printf("server:timeint upload ok......!!!\n");
			sprintf(msg485->msgbuf,"$TIMEINT_UPLOAD,ACK,EOF\n");		
			msg485->msglen 	=	strlen(msg485->msgbuf);
			iBytes = write(fd,msg485->msgbuf,msg485->msglen);
			if(iBytes == -1)
			{
				printf("Error:write  Data to slave_board\n");
			}
		}
	}
	//open lock
	else if(strncmp("$OPEN_LOCK",msg485->msgbuf,strlen("$OPEN_LOCK")) == 0)
	{
		msg485->type		= OPEN_LOCK;
		printf("rcv openlock msg\n");
		if(GprsData.process_state == SUCCESS)
		{
			printf("server:openlock ok......!!!\n");
			sprintf(msg485->msgbuf,"$OPEN_LOCK,ACK,EOF\n");		
			msg485->msglen 	=	strlen(msg485->msgbuf);
			iBytes = write(fd,msg485->msgbuf,msg485->msglen);
			if(iBytes == -1)
			{
				printf("Error:write  Data to slave_board\n");
			}
		}
	}
}

/****************************************************************************
 * master_485
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_485(int argc, char *argv[])
{
	struct timeval timeout;
	fd_set 	rfds;	
	int  	iRet = 0;
	int  	iBytes;
	
	g_485_started = true;

	fd_485 = open(CONFIG_EXAMPLES_485_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_485 < 0)
	{
		int errcode = errno;
		printf("485: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_485_DEVPATH, errcode);
		goto errout;
	}
	
	while(1)
	{
	
		FD_ZERO(&rfds);											
		FD_SET(fd_485, &rfds);									
		timeout.tv_sec = 4;
		timeout.tv_usec = 0;
		iRet = select(fd_485+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			printf("485 no senddata to master\n");
			/**********************************************************/
			//use only one master
			//add by liushuhe 2017.11.21
			GprsData.process_state = FAIL;
			/**********************************************************/
			if(GprsData.set_slavetime_flag == 1)
			{
				GprsData.set_slavetime_flag = 0;
				//send time
				sprintf(Msg485Data.msgbuf,"%s",GprsData.download_data.time);		
				Msg485Data.msglen 	=	strlen(Msg485Data.msgbuf);
				iBytes = write(fd_485,Msg485Data.msgbuf,Msg485Data.msglen);
				if(iBytes == -1)
				{
					printf("Error:write  Data to slave_board\n");
				}
			}
			//power
			if((Locker == LOCK_OK)||(Msg485Data.type == POWER_OFF))
			{
				printf("send msg: power off.....!\n");
				sprintf(Msg485Data.msgbuf,"$POWER_OFF,ASK,EOF\n");		
				Msg485Data.msglen 	=	strlen(Msg485Data.msgbuf);
				iBytes = write(fd_485,Msg485Data.msgbuf,Msg485Data.msglen);
				if(iBytes == -1)
				{
					printf("Error:write  Data to slave_board\n");
				}
			}
		}
		else
		{
			if(FD_ISSET(fd_485, &rfds)) 
			{
				usleep(100*1000L);                                     //sleep 100ms
				memset(Msg485Data.msgbuf, '\0', sizeof(Msg485Data.msgbuf));
				read(fd_485, Msg485Data.msgbuf, sizeof(Msg485Data.msgbuf));
			    tcflush(fd_485, TCIFLUSH);
				Hand_485Msg(fd_485,&Msg485Data,&adcdata);
			}
		}
	}
	    
errout:
  g_485_started = false;

  printf("485: Terminating\n");
  return EXIT_FAILURE;
}




