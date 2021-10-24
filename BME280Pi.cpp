#include "BME280Pi.h"

/*
 TODO & spi_is_busy(spiPort)
      check chipId reinit when not 0x60
      & get rid off sleeps
      detect major delta's signal caller new values available



   // spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
   // 
*/



uint8_t BME280Pi::getChipId(){

   read_registers(0xD0, &chipId, 1);
   return chipId;
}

 

bool BME280Pi::loop(void){


   // not in use ???
   if( isTimerActive(BME_TIMER_BUSY)){
      if( isWaiting(BME_TIMER_BUSY)){
         return false;
      } else {
          timerOff(BME_TIMER_BUSY);
      }
   }

   if(health>3) health=3; 
   if(health<-3) health=-3; 

   if( isTime(BME_TIMER_SENSE)
    && !spi_is_busy(spiPort) 
   ){

       read();

      if( health < -2 ){
         nextTimer(BME_TIMER_SENSE, 7);

      } else if( health < 3 ){

         nextTimerMillis(BME_TIMER_SENSE, 700);                      
      } else {

         nextTimer(BME_TIMER_SENSE, senceTiming);
      }
      return true;
   }

   return false;

}

bool BME280Pi::read(){

   bool beginSucces=true;

   if( health < 1 ){

      if(!begin()) return false;

   }  

   read_registers(0xD0, &chipId, 1);

   if( chipId != 0x60){

      health--;
      readErrors++;
      Serial.printf("Read error Chip-ID 0x%x\n", chipId);  //      ChipModel_BME280 = 0x60

      return false;
   }

   health=3;
   readCount++; 

   bme280_read_raw(&humidity, &pressure, &temperature);

   // These are the raw numbers from the chip, so we need to run through the
   // compensations to get human understandable numbers
   pressure = compensate_pressure(pressure);
   temperature = compensate_temp(temperature);
   humidity = compensate_humidity(humidity);
   //    float humidityCompensated;
   // float pressureCompensated;
   // float temperatureCompensated;

   relHum = humidity / 1024;    // convert to a fixed percentage
   
   // float temp = temperature / 100.0;  
   testhum = (humidity / 1024.0) * 4.2 * exp( (temperature/100.0) *0.06235398);
   absHum = (float)((humidity / 1024.0) * 4.2 * exp((temperature/100.0)*0.06235398));   /* in   0.01 g/m3 */ 

   // bme.read(bmePress, bmeTemp, bmeHum, tempUnit, presUnit);
   // relHum = bmeHum;
   // humTemp = bmeTemp;

   // absHum = bmeHum * 4.2 * exp(bmeTemp*0.06235398);  /* in   0.01 g/m3 */
   // press = bmePress / 100;	

   return true;
}

bool BME280Pi::begin() {

   // Chip select is active-low, so we'll initialise it to a driven-high state
   gpio_init(cs_pin);
   gpio_set_dir(cs_pin, GPIO_OUT);
   gpio_put(cs_pin, 1);

	spi_init(spiPort, 500 * 1000);

   gpio_set_function(di_pin, GPIO_FUNC_SPI);
   gpio_set_function(clk_pin, GPIO_FUNC_SPI);
   gpio_set_function(do_pin, GPIO_FUNC_SPI);

   read_registers(0xD0, &chipId, 1);

   if( chipId != 0x60){

      Serial.printf("Invalid Begin Chip-ID 0x%x\n", chipId);  //      ChipModel_BME280 = 0x60
      // started--;
      health--;
      readErrors++;
      return false;
   }
 
   read_compensation_parameters();

   write_register(0xF2, 0x1); // Humidity oversampling register - going for x1
   write_register(0xF4, 0x27);// Set rest of oversampling modes and run mode to normal

   // started++;
   health++;
   return true;
}

void BME280Pi::write_register(uint8_t reg, uint8_t data) {
   uint8_t buf[2];
   buf[0] = reg & 0x7f;  // remove read bit as this is a write
   buf[1] = data;
   gpio_put(cs_pin, 0); 
   asm volatile("nop \n nop \n nop");    

   spi_write_blocking(spiPort, buf, 2);
   asm volatile("nop \n nop \n nop");    

   gpio_put(cs_pin, 1);
   asm volatile("nop \n nop \n nop");  
   // sleep_ms(1);
}

void BME280Pi::read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
   //  For this particular device, we send the device the register we want to read
   //  first, then subsequently read from the device. The register is auto incrementing
   //  so we don't need to keep sending the register we want, just the first.
   reg |= READ_BIT;
   gpio_put(cs_pin, 0); 
   asm volatile("nop \n nop \n nop");    

   spi_write_blocking(spiPort, &reg, 1);
   asm volatile("nop \n nop \n nop");
   // sleep_ms(1);

   spi_read_blocking(spiPort, 0, buf, len);
   asm volatile("nop \n nop \n nop");   

   gpio_put(cs_pin, 1);
   asm volatile("nop \n nop \n nop");
   // sleep_ms(1);
}


/* The following compensation functions are required to convert from the raw ADC
data from the chip to something usable. Each chip has a different set of
compensation parameters stored on the chip at point of manufacture, which are
read from the chip at startup and used inthese routines.
*/
int32_t BME280Pi::compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
            >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280Pi::compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) dig_P6);
    var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t) (((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;

    var1 = (((int32_t) dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t) (p >> 2)) * ((int32_t) dig_P8)) >> 13;
    p = (uint32_t) ((int32_t) p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

uint32_t BME280Pi::compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) * (((v_x1_u32r *
                                                                                                  ((int32_t) dig_H3))
            >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                 ((int32_t) dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}
// health


/* This function reads the manufacturing assigned compensation parameters from the device */
void BME280Pi::read_compensation_parameters() {
    uint8_t buffer[26];

    read_registers(0x88, buffer, 24);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    read_registers(0xE1, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t) buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t) buffer[7];
}

void BME280Pi::bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[8];

    read_registers(0xF7, buffer, 8);
    *pressure = ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
    *humidity = (uint32_t) buffer[6] << 8 | buffer[7];
}

void BME280Pi::trace(char* id)
{
	Serial.print(F("@"));
	Serial.print(millis()/1000);
	Serial.print(F(" "));Serial.print(id);


   Serial.printf(": %d%%", relHum  );
   Serial.printf("-%.2fg/m3", absHum/100.0  );
   Serial.printf(", %dmBar", pressure / 100);
   Serial.printf(", %.2fC", temperature / 100.0);

	// Serial.print(F(", cs="));	 Serial.print(cs_pin);
	// Serial.print(F(", clk="));	 Serial.print(clk_pin);

   Serial.print(F(", chip=")); Serial.printf("0x%x", chipId);
 

	Serial.print(F(", hlth="));	 Serial.print(health);
	Serial.print(F(", #rd="));	 Serial.print(readCount);
	Serial.print(F("err:"));	 Serial.print(readErrors);   

	Serial.println();
}
