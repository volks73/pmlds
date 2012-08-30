******************************************************
* NRL-Pneumatically Modulated Liquid Delivery System *
******************************************************

Created by: Christopher R. Field <christopher.field@nrl.navy.mil>

The Naval Research Laboratory-Pneumatically Modulated Liquid Delivery System (NRL-PMLDS)
is designed to deliver a stable, constant liquid flow rate by monitoring the flow rate
from a flow meter and regulating the pressure in a pressure vessel to achieve a desired
flow rate. The feedback mechanism uses a Partial-Integral-Differentional (PID) 
algorithm. An electronic pressure control (EPC) unit regulates the pressure inside
a pressure vessel with a control voltage from 0-5V analog. The flow meter communicates
via the RS-232 protocol with the microcontroller (Arduino UNO or ATmega-similar chip).
Flow rate is selected with two momentary-off-momentary switches (mom-off-mom) and reported
on a liquid crystal display (LCD).

A pressure-driven, pneumatic, flow control system. An electronic flow meter with RS-232 
communication is used as a feedback to adjust the pressure to maintain a constant flow 
in the uL/min range. An electronic pressure control (EPC) unit is used to control the 
pressure. It has a 0-5V input range with 5V equal to the maximum controllable pressure. 
The EPC is only capable of controlling pressure to 10% its maximum value. The user can 
adjust the target flow rate using two Mom-Off-Mom switches. One switch is for the 1's 
position and the second switch is for the 10's position, with a maximum flow rate of 1000 
uL/min and a minimum flow rate of 10 uL/min. However, at 10 uL/min the flow meter maybe 
unstable. Depending the tubing inner diameter, EPC maximum range, and visocity of the 
liquid, a flow rate of 1000 uL/min may not be possibleA LCD reports the target flow rate 
(TF) in uL/min, the actual flow rate (AF) in ul/min, the EPC control voltage (V) in volts,
and the pressure (P) in PSI.
 
A single channel 8-bit digital-to-analog converter IC (MAX517) is used to output a control 
voltage between 0-5V. A TTL-to-RS232 converter IC (MAX232) is used to communicate to the 
Sensirion flow meter. See the datasheets for each IC to determine the electrical 
connections. Communication with the MAX517 is accomplished via I2C protocol on Analog 4 
and 5 with the Wire Library. The flow meter communicates at a 19200 baud rate, which can 
only be accomplished on the Digital 0 and 1 channels. Thus, uploading to the 
microcontroller while communicating with the flow meter is impossible.
 
The pressure is not actively monitored by the microcontroller, instead it is calculated 
from the control voltage based on the full range maximum of the EPC unit. The EPC unit 
can only control the pressure to 10% its maximum value. Any pressure below the 10% of 
its full range is consider 0 PSI.

== Installation ==

Download and install the Arduino Integrated Development Environment (IDE) version 1.0 or 
newer prior to proceeding with the installation. Once the IDE is installed, follow the
directions from the IDE and the Arduino website (http://www.arduino.cc) for installing
the Uno drivers and connecting an Arduino Uno to a computer for uploading code.

Once the IDE and drivers are installed and an Arduino Uno is connected to a computer via
the USB port, copy the ASL1600 and PID libraries to the library folder of the Arduino
IDE. The library folder is typically located at C:\path\to\IDE\libraries for the
Windows operating system. In other words, the libraries folder is located in the same
folder as the arduino.exe file. Make sure to copy both folders (ASL1600 and PID) to the
library folder. Restart the IDE. The two libraries should now be present in the list
of libraries in the IDE. If not, review the steps available with the IDE for installing
and using user-created libraries.

Select the communications (COM) port the Arduino Uno is connected to within the IDE. Make
sure all jumpers and shields have been removed the circuits and the Uno prior to 
uploading. Uploading code to the Uno is a delicate process and extra circuits and 
electrical connections make disrupt the upload process. Open the PMLDS.ino file within the
IDE and upload the code to the microcontroller. For additional details on uploading
code, see the IDE documentation or the Arduino website (http://www.arduino.cc).

Once the code has been successfully uploaded to the controller, the PMLDS-specific 
circuit board, or shield, can be mounted on top of the Uno circuit board. The board and
controller are then ready to be mounted in a housing with the appropriate switches, 
power supply, LCD screen, etc.

== Versions ==

Verision 2.0.0 adds computer communication to the PMLDS system. The computer communicates with the
PMLDS via the RS-232 protocol on a Serial port at 9600 Baud Rate, 8 data bits, 1 stop bit, no parity,
and no hardware flow control. The commands are listed below:
 
T=##.# Sets the target flow in uL/min
T?     Gets the target flow rate in uL/min
A?     Gets the average flow rate in uL/min
I?     Gets the instant flow rate in uL/min, will be noisy compared to A? command
V=#.## Sets the EPC control voltage in volts, note that PID control must be paused
V?     Gets the EPC control voltage in volts
P=##.# Sets the pressure in PSI, note that PID control must be paused
P?     Gets the pressure in PSI
||     Pause PID Control
|>     Resume PID Control
 
All commands must be terminated with a newline character (\n), will return "ERROR" if the termination
character (\n) was not received before SERIAL_TIMEOUT (1 second). 
 
Verison 2.0.2 updates the PID constants for the PMLDS system. In previous versions, the pressure 
is updated to often and to large of swings to really maintain a constant flow over even short
periods of time. The large dead volume of the pressure vessel (100 PSI refillable spray paint
can) acts as a capacitor and does not change the pressure as often or as large as the EPC is
changing the pressure. For the nebulizer, the pressure does not require large changes. This
version is designed to work with PCB-based hardware, not the prototype wire-wrap board.
 
Verison 2.1.0 adds a flow rate read queue to average out some of the noise from the flow meter. 
The flow meter at flow rates less than 100 mL/min (ASL1600) appears to lose precision and the
noise is raised. To combat this, a 10 point rolling average of non-zero values has been added
that is used to display the flow rate to the LCD and input into the PID control. The "I?" 
command has been added to obtain the instant flow rate and the "A?" command now returns the
average flow rate. The PID constants have also been adjusted to better control the flow rate
based on the average instead of the instant value.
 
Version 2.2.0 cleans up the serial communication commands and implementation for the computer
and removes the purge option. Based on adjustment of PID constants and updated ASL1600
communication code, a purge option is not really needed and opens up space for more options
in the future.
 
Version 2.2.1 sets the default flow rate to 40 uL/min.
 
Version 2.3.0 adds support for saving PID constants and default target flow rates to EEPROM for storage
of values during power off and power on. EEPROM writes are limited to 100,000 cycles, so do not
write new values excessively. This version also adds the serial communication commands listed below
to those implemented in version 2.0.0 of the code.

KP=#.######	Sets the Kp PID constant, unitless
KP?		Gets the Kp PID constant, unitless
KI=#.######	Sets the Ki PID constant, unitless
KI?		Gets the Ki PID constant, unitless
KD=#.######	Sets the Kd PID constant, unitless
KD?		Gets the Kd PID constant, unitless
DF=##.#		Sets the default flow rate in uL/min
DF?		Gets the default flow rate in uL/min

Scientific notation support has been added for all commands where value is being set, whereever #.#####
appear in the above listing of commands including commands from Version 2.0.0. The commands have also
changed slightly for setting and getting the target flow (TF), average flow (AF), and instant flow (IF).
 
Version 3.0.0 adds support for a circuit that is not a shield for the Arduino Uno, but uses
the Atmega368 Atmel microcontroller with the Uno bootloader burned into it (Sparkfun, DEV-10524).
The all-in-one circuit board includes two voltage regulators for 9V and 5V supplies, a 4-layer
PCB with VCC and GND supply planes and added support to read the pressure directly from the
EPC. The Arduino Uno is not used for this circuit but can be used as a bootloader and code
uploader for the microcontroller.

Version 3.0.1 adds 3 second delay during power up to let the flow meter warm up and displays a 
"warming up" message on the LCD screen.