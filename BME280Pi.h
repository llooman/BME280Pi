/*

BME280Pi.h

Based on https://github.com/raspberrypi/pico-examples/blob/master/spi/bme280_spi/bme280_spi.c 

 */

#ifndef BME280PI_H
#define BME280PI_H

#include "Arduino.h"
#include "MyTimers.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define READ_BIT 0x80

#define BME_TIMER_COUNT 4
#define BME_TIMER_BUSY 0 
#define BME_TIMER_READ 1 
#define BME_TIMER_WRITE 2 
#define BME_TIMER_SENSE 3 
 
/*****************************************************************/
/* ENUMERATIONS                                                  */
/*****************************************************************/
/*
   enum MODE { MODE_SLEEP = 0b00, MODE_FORCED = 0b01, MODE_NORMAL = 0b11 };
*/
   // enum TempUnit
   // {
   //    TempUnit_Celsius,
   //    TempUnit_Fahrenheit
   // };

   // enum PresUnit
   // {
   //    PresUnit_Pa,
   //    PresUnit_hPa,
   //    PresUnit_inHg,
   //    PresUnit_atm,
   //    PresUnit_bar,
   //    PresUnit_torr,
   //    PresUnit_psi
   // };

   // enum OSR
   // {
   //    OSR_Off =  0,
   //    OSR_X1  =  1,
   //    OSR_X2  =  2,
   //    OSR_X4  =  3,
   //    OSR_X8  =  4,
   //    OSR_X16 =  5
   // };

   // enum Mode
   // {
   //    Mode_Sleep  = 0,
   //    Mode_Forced = 1,
   //    Mode_Normal = 3
   // };

   // enum StandbyTime
   // {
   //    StandbyTime_500us   = 0,
   //    StandbyTime_62500us = 1,
   //    StandbyTime_125ms   = 2,
   //    StandbyTime_250ms   = 3,
   //    StandbyTime_50ms    = 4,
   //    StandbyTime_1000ms  = 5,
   //    StandbyTime_10ms    = 6,
   //    StandbyTime_20ms    = 7
   // };

   // enum Filter
   // {
   //    Filter_Off = 0,
   //    Filter_2   = 1,
   //    Filter_4   = 2,
   //    Filter_8   = 3,
   //    Filter_16  = 4
   // };

   // enum SpiEnable
   // {
   //    SpiEnable_False = 0,
   //    SpiEnable_True = 1
   // };

   // enum ChipModel
   // {
   //    ChipModel_UNKNOWN = 0,
   //    ChipModel_BMP280 = 0x58,
   //    ChipModel_BME280 = 0x60
   // };



//////////////////////////////////////////////////////////////////
/// BME280 - Driver class for Bosch Bme280 sensor
///
/// Based on the data sheet provided by Bosch for
/// the Bme280 environmental sensor.
///
class BME280Pi: public MyTimers
{
public:
	BME280Pi(int port, uint cs, uint clk, uint spiTx, uint spiRx):MyTimers(BME_TIMER_COUNT){
      if(port==0) spiPort = spi0;
      if(port==1) spiPort = spi1; 
		cs_pin = cs;
      clk_pin = clk;
      do_pin = spiTx;
      di_pin = spiRx;
      setup();
	}
	BME280Pi( uint cs, uint clk, uint spiTx, uint spiRx):MyTimers(BME_TIMER_COUNT){
      spiPort = spi0;  // default to spi0
		cs_pin = cs;
      clk_pin = clk;
      do_pin = spiTx;
      di_pin = spiRx;
      setup();
	}   
   BME280Pi():MyTimers(BME_TIMER_COUNT){ spiPort = spi0; setup();}
	virtual ~BME280Pi(){}  // suppress warning

	void setup()
	{
      nextTimerMillis(BME_TIMER_SENSE, 50);
	}


   spi_inst_t *spiPort;

   int32_t humidity;
   int32_t pressure;
   int32_t temperature;

   float testhum;
 

   int32_t relHum=0;
   int32_t absHum=0;
 

   uint  di_pin=16;  //MISO  rx <-----> bme.tx slave
   uint  cs_pin=17;  // wit 
   uint clk_pin=18;  // zwart
   uint  do_pin=19;  //MOSI  tx <-----> bme.rx slave

	uint8_t chipId=0;
   // int     started=0;
   int     health=0;
   uint    readCount=0;
   uint    readErrors=0;

   uint    senceTiming=21;

int (*uploadFunc) (int id, long val, unsigned long timeStamp) = 0;
void onUpload( int (*function)(int id, long val, unsigned long timeStamp) )
{
uploadFunc = function;
}

int (*errorFunc) (int id, long val ) = 0;
void onError( int (*function)(int id, long val ) )
{
errorFunc = function;
}

void write_register(uint8_t reg, uint8_t data); 
void read_registers(uint8_t reg, uint8_t *buf, uint16_t len);
void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature); 

int32_t compensate_temp(int32_t adc_T);
uint32_t compensate_pressure(int32_t adc_P);   
uint32_t compensate_humidity(int32_t adc_H);

void read_compensation_parameters(void);

uint8_t getChipId(void);
bool begin(void);
 
bool read(void);
void trace(char* id);
bool loop(void);

// int upload(int id);


private:
   int32_t         t_fine;

   uint16_t        dig_T1;
   int16_t         dig_T2, dig_T3;
   uint16_t        dig_P1;
   int16_t         dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
   uint8_t         dig_H1, dig_H3;
   int8_t          dig_H6;
   int16_t         dig_H2, dig_H4, dig_H5; 

/*****************************************************************/
/* CONSTANTS                                                     */
/*****************************************************************/

   static const uint8_t CTRL_HUM_ADDR   = 0xF2;
   static const uint8_t CTRL_MEAS_ADDR  = 0xF4;
   static const uint8_t CONFIG_ADDR     = 0xF5;
   static const uint8_t PRESS_ADDR      = 0xF7;
   static const uint8_t TEMP_ADDR       = 0xFA;
   static const uint8_t HUM_ADDR        = 0xFD;
   static const uint8_t TEMP_DIG_ADDR   = 0x88;
   static const uint8_t PRESS_DIG_ADDR  = 0x8E;
   static const uint8_t HUM_DIG_ADDR1   = 0xA1;
   static const uint8_t HUM_DIG_ADDR2   = 0xE1;
   static const uint8_t ID_ADDR         = 0xD0;

   static const uint8_t TEMP_DIG_LENGTH         = 6;
   static const uint8_t PRESS_DIG_LENGTH        = 18;
   static const uint8_t HUM_DIG_ADDR1_LENGTH    = 1;
   static const uint8_t HUM_DIG_ADDR2_LENGTH    = 7;
   static const uint8_t DIG_LENGTH              = 32;
   static const uint8_t SENSOR_DATA_LENGTH      = 8;
};

#endif // TG_BME_280_H
