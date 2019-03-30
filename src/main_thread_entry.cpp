/*********************************************************************
This is the main thread for placing Arduino Sketch code to run on Renesas AE Cloud2 S5D9 board

Created on: September 17, 2018
First Released on: March 19, 2019
Author: Michael Li (michael.li@miketechuniverse.com)

An Arduino sketch is placed inside the section that is defined by two comment lines.  The section has the
example of setup() and loop() functions.   You can replace them with your example code.  It is very simple.
Most common functions like Serial and Wire functions are supported except of the SPI functions which need to
be modified to run.


The MIT License (MIT)


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


*********************************************************************/



#include "main_thread.h"
#include <Arduino.h>
#include <Wire.h>
#include <Wire1.h>
#include "SERIAL.h"
#include "SERIAL1.h"
#include "SPI.h"
#include "SPI1.h"
#include <stdio.h>

SERIAL1 Serial  = SERIAL1();   //UART 1 Serial communication
WIRE1 Wire = WIRE1();          //needed by sht31.  Use i2c grove port.

//====================== Your Arduino Example Sketch Begin ===========//

#include "SHT31.h"   // *** Need to use WIRE1 to instantiate Wire in SHT31.cpp.
SHT31 sht31 = SHT31();

void setup() {
    Serial.begin(9600);
    //while(!Serial);  // Not supported
    Serial.println("begin...");
    sht31.begin();
}

void loop() {
    float temp = sht31.getTemperature();
    float hum = sht31.getHumidity();
    Serial.print("Temp = ");
    Serial.print(temp);
    Serial.println(" C"); //The unit for  Celsius because original arduino don't support speical symbols
    Serial.print("Hum = ");
    Serial.print(hum);
    Serial.println("%");
    Serial.println();
    delay(1000);
}

//====================== Your Arduino Example Sketch End ===========//



//============================= Renesas Main Thread entry function ====================================//
void main_thread_entry(void)
{
   system_setup();
   setup();

    while (1) {
        loop();
        delay(1);  // required for the thread if delay()does not exist in the loop() function.
    }
}

