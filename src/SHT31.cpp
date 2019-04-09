/*
 *
http://wiki.seeedstudio.com/Grove-TempAndHumi_Sensor-SHT31/

The MIT License (MIT)

Copyright (c) 2013 Seeed Technology Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "SHT31.h"
#include "WIRE1.h"
#include "WIRE.h"
#include "Arduino.h"

// -------------------------------
// Added by Michael (to
// -------------------------------
#include "SERIAL1.h"
extern SERIAL1 Serial;   //UART 1 Serial communication


#define  POLYBYTE  0x31  // SHT31 sensor

#define  CRC_INITIAL 0xFF
#define  CRC_FINALXOR 0x00
unsigned char gen_crc(unsigned char *array, int len);
// -------------------------------

// use this declaration for this i2c device that owns the i2c port exclusively
//WIRE1 Wire = WIRE1(); // grove ic21 header
//WIRE Wire = WIRE();

// use this declaration for two i2c devices that share the same i2c port.
extern  WIRE1 Wire;  // grove i2c port
//extern WIRE Wire;  // Arduino header port ADC4/ADC5






SHT31::SHT31() {
}

boolean SHT31::begin(uint8_t i2caddr) {
  Wire.begin();
  _i2caddr = i2caddr;
  reset();
  //return (readStatus() == 0x40);
  return true;
}

float SHT31::getTemperature(void) {
  if (! getTempHum()) return NAN;
  return temp;
}


float SHT31::getHumidity(void) {
  if (! getTempHum()) return NAN;
  return humidity;
}

uint16_t SHT31::readStatus(void) {  
    return 0;
}

void SHT31::reset(void) {
  writeCommand(SHT31_SOFTRESET);
  delay(10);
}

void SHT31::heater(boolean h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
}

uint8_t SHT31::crc8(const uint8_t *data, int len) {
  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);
  
  for ( int j = len; j; --j ) {
      crc ^= *data++;

      for ( int i = 8; i; --i ) {
	crc = ( crc & 0x80 )
	  ? (crc << 1) ^ POLYNOMIAL
	  : (crc << 1);
      }
  }
  return crc; 
}


boolean SHT31::getTempHum(void) {
  uint8_t readbuffer[6];

  writeCommand(SHT31_MEAS_HIGHREP);
  
  delay(50);
  Wire.requestFrom(_i2caddr, (uint8_t)6);
  if (Wire.available() != 6) 
    return false;
  for (uint8_t i=0; i<6; i++) {
    readbuffer[i] = Wire.read();
  }

  // -------------------------------
  // Added by Michael
  // -------------------------------
  // Temp MSB, Temp LSB, Temp CRC
  // HUMD MSB, HUMD LSB, HUMD CRC
  unsigned char crc = 0;

  Serial.print("Temperature Data read with CRC8:");
  for (uint8_t i=0; i<3; i++) {
      Serial.print(readbuffer[i],HEX);
      Serial.print(" ");
  }
  Serial.println(" ");

  crc = gen_crc(readbuffer,2);
  Serial.print("Calculated Temperature CRC:");
  Serial.print(crc,HEX);

  if (crc == readbuffer[2])
      Serial.println(" [Pass matched CRC]");
  else
      Serial.println(" [Fail mismatched CRC]");

  Serial.print("Humitity Data read with CRC8:");
  for (uint8_t i=3; i<6; i++) {
      Serial.print(readbuffer[i],HEX);
      Serial.print(" ");
  }
  Serial.println(" ");

  crc = gen_crc(readbuffer+3,2);
  Serial.print("Calculated Humidity CRC:");
  Serial.print(crc,HEX);
  if (crc == readbuffer[5])
      Serial.println(" [Pass matched CRC]");
  else
      Serial.println(" [Fail mismatched CRC]");
  Serial.println(" ");
  // -------------------------------
  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];

  if (readbuffer[2] != crc8(readbuffer, 2)) return false;

  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  if (readbuffer[5] != crc8(readbuffer+3, 2)) return false;
 
  double stemp = ST;
  stemp *= 175;
  stemp /= 0xffff;
  stemp = -45 + stemp;
  temp = stemp;
  
  double shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  
  humidity = shum;
  
  return true;
}

void SHT31::writeCommand(uint16_t cmd) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();      
}


// This function is to process a single data byte and generate an
// eight bit CRC checksum.
//
// Input: Databyte
// Return: CRC byte
//
unsigned char gen_crc(unsigned char *array, int len) {

    unsigned char crc = CRC_INITIAL;

    for (int index=0; index < len; index++) {
        crc = crc ^ array[index];
        for (int i=0; i<8; i++) {
            if ((crc & 0x80) != 0) {     // Detect '1' in the MSb position
                crc = (crc << 1) ^ POLYBYTE;  // shift in a padding zero and shift the MSb '1' bit
            } else {
                crc = (crc << 1);
            }
            //printf("i = %d \tcrc = %2X  \n",i, (crc & 0xFF));
        }
    }
    return crc ^ CRC_FINALXOR;
}
