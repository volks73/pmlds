/*
  PID.cpp - A simple PID Controller library.
  Created by: Christopher R. Field <christopher.field@nrl.navy.mil, cfield2@gmail.com>
  Version: 2.1.1

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
   
  General rule for tuning:
  
  With increase in K_P, K_I, or K_D, the following occurs:
  +-----+-----------+-----------+---------------+-----------+
  |     | Rise Time | Overshoot | Settling Time | S-S Error |
  +-----+-----------+-----------+---------------+-----------+
  | K_P | Decreases | Increases | N/A           | Decreases |
  | K_I | Decreases | Increases | Increases     | N/A       |
  | K_D | N/A       | Decreases | Decreases     | N/A       |
  +-----+-----------+-----------+---------------+-----------+
*/
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "PID.h"

/*
	Constructor for the PID controller.
*/
PID::PID(float Kp, float Ki, float Kd, int maxOutput, int minOutput)
{
	_lastCompute = millis();
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_maxOutput = maxOutput;
	_minOutput = minOutput;
	_integral = 0.0;
	_previousError = 0.0;
}

/*
	Restarts the PID calculations.
*/
void PID::restart()
{
	_lastCompute = millis();
	_integral = 0.0;
	_previousError = 0.0;
}

/*
	Set the Kp (proportionality) constant for the PID algorithm.
*/
void PID::setKp(float Kp)
{
	_Kp = Kp;
}

/*
	Sets the Ki (integral) constant for the PID algorithm.
*/
void PID::setKi(float Ki)
{
	_Ki = Ki;
}

/*
	Sets the Kd (derivative) constant for the PID algorithm.
*/
void PID::setKd(float Kd)
{
	_Kd = Kd;
}

/*
	Computes the output value based on the target value and the current value. 
	The output will be clipped to "maxOutput" and "minOutput".
*/
int PID::compute(float target, float actual)
{
	unsigned long now = millis();
	
	int dt = now - _lastCompute;
	float error = target - actual;
	_integral = _integral + (error * dt);
	float derivative = (error - _previousError) / (float)dt;
	int output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);
	
	if ( output > _maxOutput )
	{
		output = _maxOutput;
	}
	
	if ( output < _minOutput )
	{
		output = _minOutput;
	}
	
	_previousError = error;
	_lastCompute = now;
	
	return output;
}