/*
 Created By: Christopher R. Field <christopher.field@nrl.navy.mil>
 Verison: 2.2.0
 
 A pressure-driven, pneumatic, flow control system. An electronic flow meter with RS-232 communication
 is used as a feedback to adjust the pressure to maintain a constant flow in the uL/min range. An 
 electronic pressure control (EPC) unit is used to control the pressure. It has a 0-5V input range
 with 5V equal to the maximum controllable pressure. The EPC is only capable of controlling pressure
 to 10% its maximum value. The user can adjust the target flow rate using two Mom-Off-Mom switches. One 
 switch is for the 1's position and the second switch is for the 10's position, with a maximum flow rate
 of 100 uL/min and a minimum flow rate of 10 uL/min. However, at 10 uL/min the flow meter maybe
 unstable. A LCD reports the target flow rate (TF) in uL/min, the actual flow rate (AF) in ul/min, the EPC 
 control voltage (V) in volts, and the pressure (P) in PSI.
 
 A single channel 8-bit digital-to-analog converter IC (MAX517) is used to output a control voltage between
 0-5V. A TTL-to-RS232 converter IC (MAX232) is used to communicate to the Sensirion flow meter. See the
 datasheets for each IC to determine the electrical connections. Communication with the MAX517 is accomplished
 via I2C protocol on Analog 4 and 5 with the Wire Library. The flow meter communicates at a 19200 baud rate,
 which can only be accomplished on the Digital 0 and 1 channels. Thus, uploading to the microcontroller while
 communicating with the flow meter is impossible.
 
 The pressure is not actively monitored by the microcontroller, instead it is calculated from the control
 voltage based on the full range maximum of the EPC unit.
 
 The EPC unit can only control the pressure to 10% its maximum value. Any pressure below the 10% of its
 full range is consider 0 PSI.

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
 
 All commands must be terminated with a newline character (\n).
 
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
 in the future. Flow rates, pressures, and voltages are no longer stored at integers or 10x
 their actual values. Floats are used instead since space is no longer an issue.
 */
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ASL1600.h>
#include <PID.h>

/*
  The MAX517 1-CH DAC has an address of 0x5A, or 01011010b, but the Wire library uses 7-bits and appends
 the last bit of 0 to the address, so the device address is 0x5A, but the Arduino-Wire library address
 is 0x2D, or 00101101b.
 */
const int DAC_ADDRESS = 0x2D;
const byte COMMAND = 0;
const int MAX_DAC_NUMBER = 255;
const int MIN_DAC_NUMBER = 0;
const float MAX_VOLTAGE = 5.0; // volts
const float MAX_PRESSURE = 15.0; // psi

/*
  LCD pins
 */
const int LCD_RS = 7;
const int LCD_ENABLE = 6;
const int LCD_D4 = 5;
const int LCD_D5 = 4;
const int LCD_D6 = 3;
const int LCD_D7 = 2;

/*
  LCD constants.
 */
const int TARGET_FLOW_RATE_ROW = 0;
const int ACTUAL_FLOW_RATE_ROW = 1;
const int LCD_NUM_COLS = 16;
const int LCD_NUM_ROWS = 2;

/*
  One's and ten's place increment/decrement switches pins.
 */
const int INCREMENT_BUTTON_ONE = 8;
const int DECREMENT_BUTTON_ONE = 9;
const int INCREMENT_BUTTON_TEN = 10;
const int DECREMENT_BUTTON_TEN = 11;

/*
  Flow sensor constants.
 */
const int MAX_CONNECTION_ATTEMPTS = 3;
const float MAX_FLOW_RATE = 1000; // uL/min
const float MIN_FLOW_RATE = 10; // uL/min
const float DEFAULT_FLOW_RATE = 50; // uL/min
const int FLOW_RATE_QUEUE_SIZE = 10;

/*
  PID constants.
 */
const float K_P = 0.0000005; // Previous value: 0.0000001
const float K_I = 0.00001; // Previous value: 0.000001
const float K_D = 0.01; // Previous value: 0.1
const long PID_DELAY = 2500; // milliseconds

/*
  CPU Serial Communication constants.
 */
const int RX_PIN = 12;
const int TX_PIN = 13;
const int CPU_BAUD_RATE = 9600;
const char GET_COMMAND = '?';
const char SET_COMMAND = '=';
const int ASCII_ZERO = 48;
const char TERMINATION_CHAR = '\n';
const unsigned long SERIAL_TIMEOUT = 1000; // milliseconds

float targetFlowRate = DEFAULT_FLOW_RATE; // uL/min
float instantFlowRate = MIN_FLOW_RATE; // uL/min
float averageFlowRate = MIN_FLOW_RATE; // uL/min
float flowRateQueue[FLOW_RATE_QUEUE_SIZE];
int dacNumber = MIN_DAC_NUMBER;
int incrementOneState = HIGH;
int decrementOneState = HIGH;
int incrementTenState = HIGH;
int decrementTenState = HIGH;
int prevIncrementOneState = HIGH;
int prevDecrementOneState = HIGH;
int prevIncrementTenState = HIGH;
int prevDecrementTenState = HIGH;
unsigned long lastUpdate = 0; // milliseconds
unsigned long currentTime = 0; // milliseconds

boolean pidActivated = true;
boolean flowMeterConnected = false;

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ASL1600 flowMeter;
PID flowPID(K_P, K_I, K_D, MAX_DAC_NUMBER, MIN_DAC_NUMBER);
SoftwareSerial cpuSerial(RX_PIN, TX_PIN);

void setup()
{ 
  // Initialize the flow rate queue
  for ( int i = 0; i < FLOW_RATE_QUEUE_SIZE; i++ )
  {
    flowRateQueue[i] = 0; 
  }

  // Setup the input for the 10's and 1's switch
  pinMode(INCREMENT_BUTTON_ONE, INPUT);
  pinMode(DECREMENT_BUTTON_ONE, INPUT);
  pinMode(INCREMENT_BUTTON_TEN, INPUT);
  pinMode(DECREMENT_BUTTON_TEN, INPUT);

  // Start the various components
  lcd.begin(LCD_NUM_COLS, LCD_NUM_ROWS);
  Wire.begin();
  cpuSerial.begin(CPU_BAUD_RATE);

  // Set the output voltage to zero to start
  writeDAC(MIN_DAC_NUMBER);

  // Print some startup information to the user.
  // Naval Research Laboratory (NRL)
  // Pneumatically Modulated Liquid Delivery System (PMLDS)
  lcd.print("NRL-PMLDS v2.2.0");
  lcd.setCursor(0, 1);
  lcd.print("Connecting");

  // Delay 3 seconds to give the flow meter time to warm up.
  delay(3000);

  /*
   If the flow meter and Arduino are turned on at the same time, the Arduino will check for connection
   before the flow meter has warmed up, resulting in not a connection. The Arduino could be reset, but
   instead check for connectivity with the flow meter three times before concluding a disconnect.
   */
  int attempts = 1;
  int flowFactor = -1;
  while ( attempts <= MAX_CONNECTION_ATTEMPTS && !flowMeterConnected )
  {
    flowMeter.begin();
    flowFactor = flowMeter.getFlowFactor();

    if ( flowFactor == -1 )
    {
      lcd.print(".");
      attempts++;

      // Wait a little bit before attempting another connection check.
      delay(2000);
    }
    else
    {
      flowMeterConnected = true;
      break;
    }
  }

  lcd.setCursor(0, 1);

  if ( flowMeterConnected )
  {
    // Report the flow factor as an indicator it is working
    lcd.print("Flow Factor: ");
    lcd.print(flowFactor);

    delay(3000);

    startPID();
  }
  else
  {
    lcd.clear();
    lcd.print("Flow Meter");
    lcd.setCursor(0, 1);
    lcd.print("Not Connected");
  }
}

void loop()
{
  incrementOneState = digitalRead(INCREMENT_BUTTON_ONE);  
  decrementOneState = digitalRead(DECREMENT_BUTTON_ONE);
  incrementTenState = digitalRead(INCREMENT_BUTTON_TEN);
  decrementTenState = digitalRead(DECREMENT_BUTTON_TEN);

  if ( incrementOneState != prevIncrementOneState )
  {
    if ( !incrementOneState )
    {
      targetFlowRate = targetFlowRate + 1;
    }

    prevIncrementOneState = incrementOneState;
  }

  if ( incrementTenState != prevIncrementTenState )
  {
    if ( !incrementTenState )
    {
      targetFlowRate = targetFlowRate + 10;
    }

    prevIncrementTenState = incrementTenState;
  }

  if ( decrementOneState != prevDecrementOneState )
  {
    if ( !decrementOneState )
    {
      targetFlowRate = targetFlowRate - 1; 
    }

    prevDecrementOneState = decrementOneState;
  }

  if ( decrementTenState != prevDecrementTenState )
  {
    if ( !decrementTenState )
    {
      targetFlowRate = targetFlowRate - 10; 
    }

    prevDecrementTenState = decrementTenState;
  }

  if ( targetFlowRate > MAX_FLOW_RATE )
  {
    targetFlowRate = MAX_FLOW_RATE;
  }

  if ( targetFlowRate < MIN_FLOW_RATE )
  {
    targetFlowRate = MIN_FLOW_RATE; 
  }

  // Update the screen and control voltage depending on the activity, but only if
  // the flow meter is connected.
  if ( flowMeterConnected )
  {
    // The read function must be called each iteration of the loop method.
    flowMeter.read();

    if ( pidActivated )
    {
      updateLCDFlowRate(targetFlowRate, TARGET_FLOW_RATE_ROW);
    }

    currentTime = millis();

    // Update only after a small delay, we do not need to be updating
    // every cycle.
    if ( (currentTime - lastUpdate) > PID_DELAY )
    {
      instantFlowRate = flowMeter.getFlowRate();

      // Bound the instant flow rate between the minimum and maximum flow rates of the
      // flow meter.
      if ( instantFlowRate < MIN_FLOW_RATE )
      {
        instantFlowRate = 0;
      }

      if ( instantFlowRate > MAX_FLOW_RATE )
      {
        instantFlowRate = MAX_FLOW_RATE; 
      }

      // Get the average of the last FLOW_RATE_QUEUE_SIZE non-zero instant flow rate
      // measurements. Use the average flow rate to calculate the pressure via the
      // PID control. This reduces excessive pressure control changes due to noise
      // in the flow rate reading.
      averageFlowRate = getAverageFlowRate(instantFlowRate);

      updateLCDFlowRate(averageFlowRate, ACTUAL_FLOW_RATE_ROW);

      if ( pidActivated )
      {
        dacNumber = flowPID.compute(targetFlowRate, averageFlowRate);
        writeDAC(dacNumber);
      }       

      updateLCDVoltage(dacNumber);
      updateLCDPressure(dacNumber);

      lastUpdate = millis();
    }

    // Handle the serial communication with the computer.
    if ( cpuSerial.available() )
    {
      String command = readSerialCommand();

      if ( command.startsWith("T") )
      {
        if ( command.charAt(1) == SET_COMMAND )
        {         
          targetFlowRate = readSerialTargetFlowRate(command);
        }
        else if ( command.charAt(1) == GET_COMMAND )
        {
          writeSerialFlowRate(targetFlowRate);
        }
      }
      else if ( command.startsWith("A") )
      {
        if ( command.charAt(1) == SET_COMMAND )
        {
          // Nothing, cannot set the average flow rate.  
        }  
        else if ( command.charAt(1) == GET_COMMAND )
        {
          writeSerialFlowRate(averageFlowRate);
        }
      }
      else if ( command.startsWith("I") )
      {
        if ( command.charAt(1) == SET_COMMAND )
        {
          // Nothing, cannot set the instant flow rate.
        }
        else if ( command.charAt(1) == GET_COMMAND )
        {
          writeSerialFlowRate(instantFlowRate);
        }  
      }
      else if ( command.startsWith("V") )
      {
        if ( command.charAt(1) == SET_COMMAND )
        {
          dacNumber = readSerialVoltage(command);
        }
        else if ( command.charAt(1) == GET_COMMAND )
        {
          writeSerialVoltage(dacNumber);
        }
      }
      else if ( command.startsWith("P") )
      {
        if ( command.charAt(1) == SET_COMMAND )
        {
          dacNumber = readSerialPressure(command);
        }
        else if ( command.charAt(1) == GET_COMMAND )
        {
          writeSerialPressure(dacNumber);
        }
      }
      else if ( command.startsWith("|") )
      {
        if ( command.charAt(1) == '|' )
        {
          pausePID();
        }
        else if ( command.charAt(1) == '>' )
        {
          startPID();
        }
      }
    }
  }
}

/*
  Reads the serial buffer until the TERMINATION_CHAR is reached or timeout.
 Will return "ERROR" as the command if the timeout was reached before
 the termination char was read.
 */
String readSerialCommand()
{
  String command = "";
  char readChar = 0x00;

  unsigned long timeout = millis() + SERIAL_TIMEOUT; // milliseconds

  while ( readChar != TERMINATION_CHAR && millis() < timeout )
  {
    delay(10);
    readChar = cpuSerial.read();
    command = command + readChar;
  }

  if ( readChar == TERMINATION_CHAR )
  {
    command.trim(); 
  }
  else
  {
    command = "ERROR"; 
  }

  return command;
}

/*
  Read the voltage from the cpu serial buffer and set the DAC based on the voltage. The PID control
 must be paused.
 */
int readSerialVoltage(String command)
{
  if ( !pidActivated )
  {
    float voltage = getSetValue(command);
    int dac = int((voltage / MAX_VOLTAGE) * MAX_DAC_NUMBER);

    dac = boundDACNumber(dac);

    writeDAC(dac);
    
    return dac;
  }
}

/*
  Read the pressure from the cpu serial buffer and set the DAC based on the pressure. The PID control
 must be paused.
 */
int readSerialPressure(String command)
{
  if ( !pidActivated )
  {
    float pressure = getSetValue(command);
    int dac = int((pressure / MAX_PRESSURE) * MAX_DAC_NUMBER);

    dac = boundDACNumber(dac);

    writeDAC(dac);
    
    return dac;
  } 
}

/*
  Read target flow rate from the serial communication with the computer.
 */
float readSerialTargetFlowRate(String command)
{
  float flowRate = getSetValue(command);

  if ( flowRate > MAX_FLOW_RATE )
  {
    flowRate = MAX_FLOW_RATE;  
  }
  
  if ( flowRate < MIN_FLOW_RATE )
  {
    flowRate = MIN_FLOW_RATE;  
  }

  return flowRate;
}

/*
  Gets the value to set a parameter by from the command.
*/
float getSetValue(String command)
{
  unsigned int bufferLength = command.length() - 1;
  
  char buffer[bufferLength];
  String commandValue = command.substring(2);
  commandValue.toCharArray(buffer, bufferLength);
  
  float value = atof(buffer);

  return value;
}

/*
  Limits the dac number to the minimum and maximum values.
 */
int boundDACNumber(int dac)
{
  if ( dac < MIN_DAC_NUMBER )
  {
    dac = MIN_DAC_NUMBER; 
  }

  if ( dac > MAX_DAC_NUMBER )
  {
    dac = MAX_DAC_NUMBER; 
  }

  return dac;
}

/*
  Prints a flow rate to the serial communication with the computer.
 */
void writeSerialFlowRate(float flowRate)
{ 
  cpuSerial.print(flowRate, 1);
  cpuSerial.println();
}

/*
  Prints the voltage to the serial communication with the computer.
 */
void writeSerialVoltage(int dacNumber)
{
  float voltage = convertDACtoVoltage(dacNumber);

  cpuSerial.print(voltage, 2);
  cpuSerial.println();
}

/*
  Prints the pressure in PSI to the serial communication with the computer.
 */
void writeSerialPressure(int dacNumber)
{
  float pressure = convertDACtoPressure(dacNumber);

  cpuSerial.print(pressure, 1);
  cpuSerial.println();
}

/*
  Pauses the PID control. This keeps the pressure/control voltage constant.
 */
void pausePID()
{
  pidActivated = false;

  establishPauseScreen();
}

/*
  Starts the PID control scheme.
 */
void startPID()
{
  pidActivated = true;

  establishPIDScreen();
  flowPID.restart();
}

/*
  Wipes the screen and prints the control screen. The control screen is as follows, where
 the vertical lines designate cells in the LCD screen and are not actually printed.
 
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+  
 | T | F | : |   | 3 | 0 | . | 0 |   | V | : |   | 5 | . | 0 | 0 | = Row 1
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | A | F | : |   | 3 | 0 | . | 0 |   | P | : |   | 1 | 5 | . | 0 | = Row 2
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| = Col
 */
void establishPIDScreen()
{
  lcd.clear();

  lcd.print("TF: ");
  lcd.setCursor(9, 0);
  lcd.print("V: ");
  lcd.setCursor(0, 1);
  lcd.print("AF: ");
  lcd.setCursor(9, 1);
  lcd.print("P: ");
}

/*
  Wipes the screen and prints the priming screen. The primg screen is as follows, where
 the vertical lines designate cells in the LCD screen and are not actually printed.
 
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+  
 | P | a | u | s | e | d |   |   |   | V | : |   |   | 5 | . | 0 | = Row 1
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | A | F | : |   | 3 | 0 | . | 0 |   | P | : |   | 1 | 5 | . | 0 | = Row 2
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| = Col
 */
void establishPauseScreen()
{
  lcd.clear();

  lcd.print("Paused");
  lcd.setCursor(9, 0);
  lcd.print("V:");
  lcd.setCursor(0 , 1);
  lcd.print("AF:");
  lcd.setCursor(9, 1);
  lcd.print("P: ");
}

/*
  Write the digital-to-analog converter integer number to the MAX517 chip using the I2C communication protocol.
 */
void writeDAC(int dac)
{
  Wire.beginTransmission(DAC_ADDRESS);
  Wire.write(COMMAND);
  Wire.write(dac);
  Wire.endTransmission();
}

/*
  Update a flow rate on the LCD.
 */
void updateLCDFlowRate(float flowRate, int row)
{
  int precision = 1;
  
  lcd.setCursor(4, row);

  // If the flow rate is less than 10 uL/min, print a space
  if ( flowRate < 10 )
  {
    lcd.print(" ");
  }

  // If the flow rate is greater than or equal to 100 uL/min, print an
  // extra space.
  if ( flowRate >= 100 && flowRate < MAX_FLOW_RATE )
  {
    lcd.print(" ");
    precision = 0;
  }
  else if ( flowRate >= MAX_FLOW_RATE )
  {
    precision = 0;  
  }

  lcd.print(flowRate, precision);
}

/*
  Update the voltage on the LCD.
 */
void updateLCDVoltage(int dacNumber)
{
  float voltage = convertDACtoVoltage(dacNumber);

  lcd.setCursor(12, 0);
  lcd.print(voltage, 2);
}

/*
  Update the pressure on the LCD.
 */
void updateLCDPressure(int dacNumber)
{
  float pressure = convertDACtoPressure(dacNumber);

  lcd.setCursor(12, 1);

  if ( pressure < 10 )
  {
    lcd.print(" ");
  }

  lcd.print(pressure, 1);
}

/*
  Converts a dac number to a voltage (V);
 */
float convertDACtoVoltage(int dac)
{
  return (float(dac) / float(MAX_DAC_NUMBER)) * MAX_VOLTAGE;
}

/*
  Converts a dac number to pressure (PSI).
 */
float convertDACtoPressure(int dac)
{
  return (float(dac) / float(MAX_DAC_NUMBER)) * MAX_PRESSURE;
}

/*
  Adds the flow rate to the queue and averages the values in the flow rate queue.
 */
float getAverageFlowRate(float flowRate)
{
  for ( int i = 0; i < FLOW_RATE_QUEUE_SIZE; i++ )
  {
    if ( i + 1 < FLOW_RATE_QUEUE_SIZE )
    {
      flowRateQueue[i] = flowRateQueue[i+1];
    }
    else
    {
      flowRateQueue[i] = flowRate;  
    }
  }

  float average = 0;   
  int N = 0;
  for ( int i = 0; i < FLOW_RATE_QUEUE_SIZE; i++ )
  {
    if ( flowRateQueue[i] != 0 )
    {
      average = average + flowRateQueue[i];
      N++;
    }  
  }

  if ( N == 0 )
  {
    N = 1;  
  }

  average = average / N;

  return average;
}





















