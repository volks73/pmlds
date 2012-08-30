/*
  PID.h - A simple PID Controller library.
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
*/
#ifndef PID_h
#define PID_h

class PID
{
  public:

    PID(float, float, float, int, int);
	void restart();
	void setKp(float);
	void setKi(float);
	void setKd(float);
    int compute(float, float);
	
  private:

	float _integral;
	float _Kp;
	float _Ki;
	float _Kd;
	float _previousError;
	int _maxOutput;
	int _minOutput;
	unsigned long _lastCompute;
};

#endif