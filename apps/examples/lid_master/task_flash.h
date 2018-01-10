#ifndef _TASK_FLASH_H
#define _TASK_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define		SET_TIMEA_HOUR			7	
#define		SET_TIMEA_MIN			0	
#define		SET_TIMEB_HOUR			20
#define		SET_TIMEB_MIN			0


#define		VCC_MB_DEF			7.5	
#define		VCC_SB_DEF			7.5	
#define		TEMPRETURE_DEF		32.5	
#define		HUMIDITY_DEF		90.1	
#define		CO_DEF				70.1	
#define		H2S_DEF				70.1	
#define		NH3_DEF				70.1	
#define		O2_DEF				70.1	
#define		WATER_DEF			2.5	

#define		DEV_ID				999//for test ID
struct alarm_value
{
	char msg[200];
	char write_flag;

    char timingA_hour;
    char timingA_min;
	
    char timingB_hour;
    char timingB_min;
	
	float vcc_mb;
	float vcc_sb;
	float tempretrue;
	float humidity;
	float co;
	float h2s;
	float nh3;
	float o2;
	float water;
	int user;
	int password;
	int id;
};


extern struct alarm_value  alarmdata;
/****************************************************************************
 * Private Data
 ****************************************************************************/
int master_flash(int argc, char *argv[]);

/****************************************************************************
 * Private function
 ****************************************************************************/




#ifdef __cplusplus
}
#endif

#endif 
