/* 
ScratchBoard/PicoBoard compatible sketch
for Shrimp (http://shrimping.it)
and Arduino (http://arduino.cc)

The associated design can be found on
http://fromScratchEd.nl?lang=en under a CC-BY-NC-SA License.

Copyright (C) 2013  Sjoerd Dirk Meijer (http://fromScratchEd.nl)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Original created by Koji Yokokawa (http://www.yengawa.com)
and Kazuhiro Abe (http://d.hatena.ne.jp/abee2/) (both without a license).
Sources: http://www.yengawa.com/scratch_arduino &
http://www.yengawa.com/sites/default/files/uploads/SensorBoardWithMotor.pde
*/

//		CD4052/74HC4052
//		  -----------
//	y0	  | 1	16	|	+5V
//	y2	  | 2	15	|	x2
//	aPinY | 3	14	|	x1
//	y3	  | 4	13	|	aPinX
//	y1	  | 5	12	|	x0
//	GND	  | 6	11	|	x3
//	GND	  | 7	10	|	selectPin2
//	GND	  | 8	 9	|	selectPin1
//		  -----------

// pins for CD4052
const int selectPin1 = 3;		// pin 1 connected to the 4052 input select lines
const int selectPin2 = 2;		// pin 2 connected to the 4052 input select lines
const int analogPinX = A1;      // the analog pin connected to CD4052 output - X-side
const int analogPinY = A0;		// the analog pin connected to CD4052 output - Y-side

// digital input pin
#define BUTTON 4

// analog input pin
#define SLIDER A5
#define LIGHT  A3
#define SOUND  A4
#define R_A analogPinY //connect to y0 of CD4052
#define R_B analogPinY //connect to y1 of CD4052
#define R_C analogPinX //connect to x1 of CD4052
#define R_D analogPinX //connect to x0 of CD4052

#define FIRMWAEW_ID 4  // ScratchBoard 1.1 Firmware ID

#define LED_PIN 13  // response led (digital pin 13 is on board LED)

const byte req_scratchboard = 0;  // request messge from Scratch
const byte mask_scratcharduino = 240;  // request mask of Scratch+Ardunio
const byte ch_r_D = 0;
const byte ch_r_C = 1;
const byte ch_r_B = 2;
const byte ch_button = 3;
const byte ch_r_A = 4;
const byte ch_light = 5;
const byte ch_sound = 6;
const byte ch_slider = 7;
const byte ch_firmware = 15;

int sensorValue = 0;  // sensor value to send

byte inByte = 0;  // incoming serial byte

// this function sets the CD4052
int setBytes(int byte1, int byte2)
{
	digitalWrite(selectPin1, byte1);
	digitalWrite(selectPin2, byte2);
}

void setup()
{
	// start serial port at 9600 bps:
	Serial.begin(38400);
	pinMode(BUTTON, INPUT);   // digital sensor for button
	digitalWrite(BUTTON, HIGH);

	// initialize the digital pin as an output:
	pinMode(LED_PIN, OUTPUT);
	pinMode(selectPin1, OUTPUT);
	pinMode(selectPin2, OUTPUT);
	//digitalWrite(LED_PIN, HIGH);   // set the LED on
}

void loop()
{
	// if we get a valid byte, read analog ins:
	if (Serial.available() > 0) {
		// get incoming byte:
		inByte = Serial.read();
		Serial.flush();
		if (inByte >= req_scratchboard) {
			digitalWrite(LED_PIN, HIGH);   // set the LED on

			sendValue(ch_firmware, FIRMWAEW_ID);
			delay(10);

			// analog read range from 0 to 1023
			// delay 10ms to let the ADC recover:
			
			setBytes(0, 0);  //set CD4052 to read pins x0, y0
			sensorValue = analogRead(R_A);
			sendValue(ch_r_A, sensorValue);
			delay(10);

			sensorValue = analogRead(R_D);
			sendValue(ch_r_D, sensorValue);
			delay(10);

			setBytes(0, 1); //set CD4052 to read pins x1, y1
			
			sensorValue = analogRead(R_B);
			sendValue(ch_r_B, sensorValue);
			delay(10);

			sensorValue = analogRead(R_C);
			sendValue(ch_r_C, sensorValue);
			delay(10);

			//BUTTON, map it to 0 or 1023
			sensorValue = map(digitalRead(BUTTON), 0, 1, 0, 1023);  
			sendValue(ch_button, sensorValue);

			//LIGHT
			sensorValue = analogRead(LIGHT);
			sendValue(ch_light, 1024-(sensorValue*1.2));
			delay(10);

			//SOUND
			int minimum = 1024;
			int maximum = 0;
			for(int i=0;i<1000;i++)
			{
				int value = analogRead(SOUND); 
				minimum = min(minimum, value);
				maximum = max(maximum, value);
			}
			sensorValue = maximum-minimum;
			sendValue(ch_sound, sensorValue);
			delay(10);

			//SLIDER
			sensorValue = analogRead(SLIDER);
			sendValue(ch_slider, sensorValue);
			delay(10);
		}
		digitalWrite(LED_PIN, LOW);    // set the LED off
	}
}

void sendValue(byte channel, int value) {
	byte high = 0;  // high byte to send
	byte low = 0;  // low byte to send
	high = (1 << 7) | (channel << 3) | (value >> 7);
	low =  (0xff >> 1) & value;
	Serial.write(high);
	Serial.write(low);
}
