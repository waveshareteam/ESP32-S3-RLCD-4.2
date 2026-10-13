#ifndef I2C_EQUIPMENT_H
#define I2C_EQUIPMENT_H


#include "i2c_bsp.h"

typedef enum{
  	NO_ERROR       = 0x00, // no error
  	ACK_ERROR      = 0x01, // no acknowledgment error
  	CHECKSUM_ERROR = 0x02 // checksum mismatch error
}etError;

typedef enum{
  	READ_ID            = 0xEFC8, // command: read ID register
  	SOFT_RESET         = 0x805D, // soft reset
  	SLEEP              = 0xB098, // sleep
  	WAKEUP             = 0x3517, // wakeup
  	MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
  	MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
  	MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled
  	MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
}etCommands;


class Shtc3Port
{
private:
	const char *TAG = "SHTC3";
	uint16_t shtc3_id = 0x00;
	const uint16_t CRC_POLYNOMIAL = 0x131;
	const uint8_t  SHTC3_PETP_VOL = 4;
	const uint8_t Shtc3Address = 0x70;
	I2cMasterBus& i2cbus_;
    i2c_master_dev_handle_t I2c_DevShtc3;

	etError Shtc3_GetId();
	etError Shtc3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,uint8_t checksum);
	etError Shtc3_GetTempAndHumiPolling(float *temp, float *humi);
	float Shtc3_CalcTemperature(uint16_t rawValue);
	float Shtc3_CalcHumidity(uint16_t rawValue);

public:
	Shtc3Port(I2cMasterBus& i2cbus);
	~Shtc3Port();

	etError Shtc3_Wakeup();
	etError Shtc3_Sleep();
	etError Shtc3_SoftReset();
	uint16_t Shtc3_GetShtc3Id();
	uint8_t Shtc3_ReadTempHumi(float *t,float *h);
};


typedef struct 
{
  	int year;
  	int month;
  	int day;
  	int hour;
  	int minute;
  	int second;
  	int week;
}rtcTimeStruct_t;

void Rtc_Setup(I2cMasterBus *i2cbus,uint8_t dev_addr);
void Rtc_SetTime(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second);
void Rtc_GetTime(rtcTimeStruct_t *time);

#endif 
