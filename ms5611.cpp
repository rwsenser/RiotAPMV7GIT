/*
 * ms5611.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: kbisland
 */

#include "ms5611.h"
#include "Arduino.h"
#include <SPI.h>

// fix:
#define digitalWriteFast digitalWrite

ms5611::ms5611(int cs) {
	this->_cs = cs;
	this->n_crc = 0;
}
void ms5611::init(){
  pinMode(this->_cs, OUTPUT);
	digitalWriteFast(this->_cs, HIGH);
	SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(1);  
	// cmd_reset(); // reset the module after powerup
	for (int i=0;i<8;i++){
	  C[i]=cmd_prom(i); // read calibration coefficients
	} 
	n_crc=crc4(C);
}

unsigned long ms5611::cmd_adc(char cmd){
	// SPISettings settingsA(2000000, LSBFIRST, SPI_MODE1);
  SPISettings settingsA(14000000,MSBFIRST, SPI_MODE3);
	unsigned int ret;
	unsigned long temp;
	SPI.beginTransaction(settingsA); 
	digitalWriteFast(this->_cs, LOW);
	SPI.transfer(CMD_ADC_CONV+cmd);
  digitalWriteFast(this->_cs, HIGH); 
	switch (cmd & 0x0f) // wait necessary conversion time. Max time for ADC used to be safe.
	{
	case CMD_ADC_256 : delayMicroseconds(600); break;
	case CMD_ADC_512 : delayMicroseconds(1170); break;
	case CMD_ADC_1024: delayMicroseconds(2280); break;
	case CMD_ADC_2048: delayMicroseconds(4540); break;
	case CMD_ADC_4096: delayMicroseconds(9040); break;
	}
	digitalWriteFast(this->_cs, LOW);
	digitalWriteFast(this->_cs,LOW);
	SPI.transfer(CMD_ADC_READ);
  // delay(1); // kludge 
  temp = 0;
	ret = SPI.transfer(0x00);
	temp += 65536*ret;
	ret = SPI.transfer(0x00);
	temp += (256*ret);
	ret = SPI.transfer(0x00);
	temp += ret;
	digitalWriteFast(this->_cs, HIGH);
	return temp;
}

unsigned long ms5611::getPressure(){
	long pressure = 0;
	pressure = cmd_adc(CMD_ADC_D1+CMD_ADC_256);
	return pressure;

}

double ms5611::getPressureCompensated(){
	long pressure = 0;
	double P = 0;
	pressure = cmd_adc(CMD_ADC_D1+CMD_ADC_256);
	double dT=getTemperature()-C[5]*pow(2,8);
	double OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);
	double SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7);
	P=(((pressure*SENS)/pow(2,21)-OFF)/pow(2,15))/100;
	return P;
}

unsigned long ms5611::getTemperature(){
  unsigned long temperature;
  temperature = cmd_adc(CMD_ADC_D2+CMD_ADC_4096); 
	// bob try and wrong: temperature = cmd_adc(CMD_ADC_CONV+CMD_ADC_D2+CMD_ADC_4096);
	return temperature;
}

double ms5611::getTemperatureCompensated(){
	unsigned long temperature = 0;
	double T = 0;
	temperature = getTemperature(); // cmd_adc(CMD_ADC_D2+CMD_ADC_4096);
	double dT=getTemperature()-C[5]*pow(2,8);
	double OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);
	double SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7);
	T=(2000+(dT*C[6])/pow(2,23))/100;
	return T;
}

void ms5611::cmd_reset()
{
	digitalWriteFast(this->_cs, LOW); // pull CSB low to start the command
	SPI.transfer(CMD_RESET); // send reset sequence
	// delayMicroseconds(3); // wait for the reset sequence timing
  delay(1);
	digitalWriteFast(this->_cs,HIGH); // pull CSB high to finish the command
}

unsigned int ms5611::cmd_prom(char coef_num)
{
	unsigned int ret;
	unsigned int rC=0;

	digitalWriteFast(this->_cs, LOW); // pull CSB low
	SPI.transfer(CMD_PROM_RD+coef_num*2); // send PROM READ command
	ret = SPI.transfer(0x00); // send 0 to read the MSB
	rC=256*ret;
	ret = SPI.transfer(0x00); // send 0 to read the LSB
	rC=rC+ret;
	digitalWriteFast(this->_cs, HIGH); // pull CSB high
	return rC;
}

unsigned char ms5611::crc4(unsigned int n_prom[])
{
	int cnt; // simple counter
	unsigned int n_rem; // crc reminder
	unsigned int crc_read; // original value of the crc
	unsigned char n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	{ // choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7]=crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x00);
}
