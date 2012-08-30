/*
  ASL1600.h - A library for communicating with the Sensirion ASL1600 flow meter.
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
#ifndef ASL1600_h
#define ASL1600_h

class ASL1600
{
  public:

    ASL1600();
    void begin();
	void read();
    float getFlowRate();
	int getFlowFactor();
    
  private:
    
    int _flowFactor;
	int _flowRate;
	int _syncBuffer[4];
  
    void init(char, int, int);
    void disableSecurityMode();
    void setMeasurementMode(char);
    void setDataFormat(int);
    void setSampleRate(int);
    void startMeasurement();
    void stopMeasurement();
	int info();
    void clearNBytes(int);
    void clearAllBytes();
};

#endif