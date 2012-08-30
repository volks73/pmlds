/*
  ASL1600.cpp - A library for communicating with the Sensirion ASL1600 flow meter.
  Created by: Christopher R. Field <christopher.field@nrl.navy.mil, cfield2@gmail.com>
  Version: 2.0.5
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
  
#include "ASL1600.h"

/*
  The baud rate for communicating with the ASL1600 flow meter.
*/
const int BAUD_RATE = 19200;

/*
  The zero character in ASCII decimal.
*/
const int ASCII_ZERO = 48;

/*
  Delay after sending a command to the flow meter before reading the buffer in milliseconds.
*/
const int COMMAND_DELAY = 100;

/*
  The default measurement mode, flow rate in uL/min.
*/
const char DEFAULT_MODE = 'F';

/*
  The default data format, linearized with temperature compensation.
*/
const int DEFAULT_FORMAT = 0;

/*
  The default sample rate, 1.25 Hz.
*/
const int DEFAULT_SAMPLE_RATE = 7;

/*
	The default sync buffer value.
*/
const byte DEFAULT_SYNC_BUFFER = 0x00;

/*
  The number of bytes to read while obtaining the flow rate.
*/
const int NUM_BYTES = 128;

/*
  The integer used for syncing flow rate measurements.
*/
const int SYNC_CODE = 0x7F;

/*
  The size of the flow rate measurement buffer.
*/
const int SYNC_BUFFER_SIZE = 4;

/*
  Default constructor. Sets the flow factor to -1 and the flow rate to 0. It also fills the read buffer with SYNC_CODE for each element.
*/
ASL1600::ASL1600() 
{
	_flowFactor = -1;
	_flowRate = 0.0;
	for ( int i = 0; i < SYNC_BUFFER_SIZE; i++ )
	{
	  _syncBuffer[i] = DEFAULT_SYNC_BUFFER;
	}
}

/*
  Begins communication with the ASL1600 flow meter. This sets the serial baud rate to BAUD_RATE (19200), sets the 
  flow factor based on the value returned on an 'info' command, sets the measurement mode to default ('F'), sets 
  the data format to default (0, linearized), sets the sample rate to default (3, 25 Hz), and enables measurement mode.
  
  This method should only be called once and within the 'setup()' method.
*/
void ASL1600::begin()
{
  Serial.begin(BAUD_RATE);
  stopMeasurement();
  disableSecurityMode();
  setMeasurementMode(DEFAULT_MODE);
  setDataFormat(DEFAULT_FORMAT);
  setSampleRate(DEFAULT_SAMPLE_RATE);
  _flowFactor = info();
  startMeasurement();
}

/*
  Read the flow meter buffer and sync the flow rate data to obtain a flow rate. This method should be called with each
  iteration of the 'loop()' method. Use the 'getFlowRate()' to obtain the flow rate. This method will return -1 if an
  error has occurred with reading the flow meter, typically it is not connected, will return 0 if it communicated with
  the flow meter but did not read a flow rate (read a byte, but was not synced), and will return 1 if it synced and
  obtained a flow rate.
*/
void ASL1600::read()
{
  int numBytes = Serial.available();

  if ( numBytes )
  {
    float averageFlowRate = 0.0;
    int numFlowRateReadings = 0;

    for ( int i = 0; i < numBytes; i++ )
    {
      for ( int j = 0; j < SYNC_BUFFER_SIZE; j++ )
      {
        if ( j+1 < SYNC_BUFFER_SIZE )
        {
          _syncBuffer[j] = _syncBuffer[j+1];
        }
        else
        {
          _syncBuffer[j] = Serial.read();
        }
      }

      if ( _syncBuffer[0] == SYNC_CODE && _syncBuffer[1] == SYNC_CODE && _syncBuffer[2] != SYNC_CODE )
      {
        averageFlowRate = averageFlowRate + ((int(word(_syncBuffer[2], _syncBuffer[3]))) / float(_flowFactor));
        numFlowRateReadings++;
      }
    }

    _flowRate = int((averageFlowRate / float(numFlowRateReadings)) * 10);
  }
}

/*
  Gets the flow rate in uL/min. The flow factor has already been applied.
*/
float ASL1600::getFlowRate()
{ 
  return float(_flowRate) / 10.0;
}

/*
  Gets the flow factor used to calculate the flow rate in uL/min based on the calibration of the sensor at the factory.
  Will return -1 if communication with the flow meter has not been established.
*/
int ASL1600::getFlowFactor()
{
  return _flowFactor;
}

/*
  Disables the security mode, enabling changing of the configuration. Clears the flow meter serial buffer after issuing command.
*/
void ASL1600::disableSecurityMode()
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }
  
  Serial.print("pw=expand");
  delay(COMMAND_DELAY);
  
  clearAllBytes();
}

/*
  Sets the mode, either "F" for flow rate or "T" for temperature measurements. Clears the flow meter serial buffer after issuing command.
*/
void ASL1600::setMeasurementMode(char mode)
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }
  
  Serial.print("mod=");
  Serial.print(mode);
  Serial.print("\n");
  delay(COMMAND_DELAY);
  
  clearAllBytes();
}

/*
  Sets the data mode, either "1" for raw or "0" for linearized, temperature compensated. The linearized temperature compensated data
  mode is necessary for 99% of all situations. Clears the flow meter serial buffer after issuing command.
*/
void ASL1600::setDataFormat(int format)
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }
  
  Serial.print("raw=");
  Serial.print(format);
  Serial.print("\n");
  delay(COMMAND_DELAY);
  
  clearAllBytes();
}

/*
  Sets the sample rate. A higher sample rate results in a decrease in flow rate resolution (maximum of 15 bits). Clears the flow
  meter serial buffer after issuing command.
  
  Acceptable sample rate values:
  0 = 8 bit resolution, 200 Hz
  1 = 9 bit resolution, 100 Hz
  2 = 10 bit resolution, 50 Hz
  3 = 11 bit resolution, 25 Hz
  4 = 12 bit resolution, 12.5 Hz
  5 = 13 bit resolution, 6.25 Hz
  6 = 14 bit resolution, 3.125 Hz
  7 = 15 bit resolution, 1.56 Hz
*/
void ASL1600::setSampleRate(int sampleRate)
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }

  Serial.print("res="); 
  Serial.print(sampleRate);
  Serial.print("\n");
  delay(COMMAND_DELAY);
  
  clearAllBytes();
}

/*
  Disables measurement mode. The flow meter stops reporting the flow rate.
*/
void ASL1600::stopMeasurement()
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }
  
  Serial.print("s");
  delay(COMMAND_DELAY);
  
  clearAllBytes();
}

/*
  Enables measurement mode. The flow meter enters measurement mode and reports data. The measurement mode must be disabled with the
  stopMeasurement() function before issuing other commands.
*/
void ASL1600::startMeasurement()
{
  if ( Serial.available() )
  {
    clearAllBytes();
  }
  
  Serial.print("go\n");
  delay(COMMAND_DELAY);
}

/*
  Sends the "info" command, and returns the flow factor, but no other factory settings. Will return -1 if the flow factor could not be
  determined due to unknown communication issue.
*/
int ASL1600::info()
{
  if ( Serial.available() )
  {
	clearAllBytes();
  }
	
  Serial.print("info\n");
  delay(COMMAND_DELAY);
  
  clearNBytes(54);
    
  int firstDigit = Serial.read();
  int secondDigit = Serial.read();
    
  if ( firstDigit != -1 && secondDigit != -1 )
  {
    /*
     A simple method for converting two ASCII digits into an integer value. For example, if the first digit is ASCII DEC 50 (char 2) and 
     the second digit is ASCII DEC 57 (char 9), the integer value is 29. The numbers are listed in decimal order in ASCII, so simiply 
     subtract char zero (ASCII 48) from both digits and multiple the first digit by 10.
    */
    _flowFactor = (firstDigit - ASCII_ZERO) * 10 + (secondDigit - ASCII_ZERO);
  }
  else
  {
	_flowFactor = -1;
  }
    
  clearAllBytes();
    
  return _flowFactor;
}

/*
  Clear N bytes from the flow meter serial buffer and discards the contents.
*/
void ASL1600::clearNBytes(int numBytes)
{
  for ( int i = 0; i < numBytes; i++ )
  {
    Serial.read();
	delay(10);
  }
}

/*
  Reads all bytes in the ASL1600 Serial buffer. This is not the same as Serial.flush(), which only clears the Arduino's Serial buffer.
*/
void ASL1600::clearAllBytes()
{
  while ( Serial.available() )
  {
	Serial.read();
	delay(10);
  }
}