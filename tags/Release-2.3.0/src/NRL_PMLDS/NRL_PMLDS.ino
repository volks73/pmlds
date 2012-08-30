/*
 Created By: Christopher R. Field <christopher.field@nrl.navy.mil>
 Verison: 2.3.0
 
 A pressure-driven, pneumatic, flow control system. An electronic flow meter with RS-232 communication
 is used as a feedback to adjust the pressure to maintain a constant flow in the uL/min range. An 
 electronic pressure control (EPC) unit is used to control the pressure. It has a 0-5V input range
 with 5V equal to the maximum controllable pressure. The EPC is only capable of controlling pressure
 to 10% its maximum value. The user can adjust the target flow rate using two Mom-Off-Mom switches. One 
 switch is for the 1's position and the second switch is for the 10's position, with a maximum flow rate
 of 1000 uL/min and a  minimum flow rate of 10 uL/min. However, at 10 uL/min the flow meter maybe
 unstable. Depending the tubing inner diameter, EPC maximum range, and visocity of the liquid, a flow rate 
 of 1000 uL/min may not be possibleA LCD reports the target flow rate (TF) in uL/min, the actual flow rate 
 (AF) in ul/min, the EPC control voltage (V) in volts, and the pressure (P) in PSI.
 
 A single channel 8-bit digital-to-analog converter IC (MAX517) is used to output a control voltage between
 0-5V. A TTL-to-RS232 converter IC (MAX232) is used to communicate to the Sensirion flow meter. See the
 datasheets for each IC to determine the electrical connections. Communication with the MAX517 is accomplished
 via I2C protocol on Analog 4 and 5 with the Wire Library. The flow meter communicates at a 19200 baud rate,
 which can only be accomplished on the Digital 0 and 1 channels. Thus, uploading to the microcontroller while
 communicating with the flow meter is impossible.
 
 The pressure is not actively monitored by the microcontroller, instead it is calculated from the control
 voltage based on the full range maximum of the EPC unit. The EPC unit can only control the pressure to 10% 
 its maximum value. Any pressure below the 10% of its full range is consider 0 PSI.

 Verision 2.0.0 adds computer communication to the PMLDS system. The computer communicates with the
 PMLDS via the RS-232 protocol on a Serial port at 9600 Baud Rate, 8 data bits, 1 stop bit, no parity,
 and no hardware flow control. The commands are listed below:
 
 TF=##.# Sets the target flow in uL/min
 TF?     Gets the target flow rate in uL/min
 AF?     Gets the average flow rate in uL/min
 IF?     Gets the instant flow rate in uL/min, will be noisy compared to A? command
 V=#.## Sets the EPC control voltage in volts, note that PID control must be paused
 V?     Gets the EPC control voltage in volts
 P=##.# Sets the pressure in PSI, note that PID control must be paused
 P?     Gets the pressure in PSI
 ||     Pause PID Control
 |>     Resume or restart PID Control
 
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
 DF=##.#	Sets the default flow rate in uL/min
 DF?		Gets the default flow rate in uL/min

 Scientific notation support has been added for all commands where value is being set, whereever #.#####
 appear in the above listing of commands including commands from Version 2.0.0. The commands have also
 changed slightly for setting and getting the target flow (TF), average flow (AF), and instant flow (IF).
 */
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
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
  LCD pins and constants.
 */
const int LCD_RS = 7;
const int LCD_ENABLE = 6;
const int LCD_D4 = 5;
const int LCD_D5 = 4;
const int LCD_D6 = 3;
const int LCD_D7 = 2;
const int TARGET_FLOW_RATE_ROW = 0;
const int ACTUAL_FLOW_RATE_ROW = 1;
const int LCD_NUM_COLS = 16;
const int LCD_NUM_ROWS = 2;
const int PRESSURE_PRECISION = 1;
const int VOLTAGE_PRECISION = 2;

/*
  One's and ten's place increment/decrement switches pins and constants.
 */
const int INCREMENT_ONE_PIN = 8;
const int DECREMENT_ONE_PIN = 9;
const int INCREMENT_TEN_PIN = 10;
const int DECREMENT_TEN_PIN = 11;
const int ONE_INCREMENT = 1; // uL/min
const int TEN_INCREMENT = 10; // uL/min

/*
  Flow sensor constants.
 */
const int MAX_CONNECTION_ATTEMPTS = 3;
const int MAX_FLOW_RATE = 1000; // uL/min
const int MIN_FLOW_RATE = 10; // uL/min
const int DEFAULT_FLOW_RATE = 40; // uL/min
const int FLOW_RATE_MULTIPLIER = 10; // uL/min
const int FLOW_RATE_QUEUE_SIZE = 10;
const int FLOW_RATE_PRECISION = 1;

/*
  EEPROM constants.
*/
const int NUM_BYTES_PER_FLOAT = 4;
const int DF_ADDRESS = 0;
const int KP_ADDRESS = 4;
const int KI_ADDRESS = 8;
const int KD_ADDRESS = 16;

/*
  PID constants.
 */
const float K_P = 5E-8; // Previous value: 0.0000001
const float K_I = 1E-5; // Previous value: 0.000001
const float K_D = 1E-2; // Previous value: 0.1
const unsigned long PID_DELAY = 2500; // milliseconds
const int PID_PRECISION = 3;

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

/*
  Global variables.
*/
int targetFlowRate = DEFAULT_FLOW_RATE * FLOW_RATE_MULTIPLIER; // uL/min * 10
int defaultFlowRate = DEFAULT_FLOW_RATE * FLOW_RATE_MULTIPLIER; // uL/min * 10
int instantFlowRate = MIN_FLOW_RATE; // uL/min * 10
int averageFlowRate = MIN_FLOW_RATE; // uL/min * 10
int flowRateQueue[FLOW_RATE_QUEUE_SIZE]; // uL/min * 10
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
float Kp = K_P; // unitless
float Ki = K_I; // unitless
float Kd = K_D; // unitless

boolean pidActivated = true;
boolean flowMeterConnected = false;

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ASL1600 flowMeter;
PID flowPID(Kp, Ki, Kd, MAX_DAC_NUMBER, MIN_DAC_NUMBER);
SoftwareSerial cpuSerial(RX_PIN, TX_PIN);

void setup()
{ 
  // Initialize the flow rate queue
  for ( int i = 0; i < FLOW_RATE_QUEUE_SIZE; i++ )
  {
    flowRateQueue[i] = 0; 
  }

  // Setup the input for the 10's and 1's switch
  pinMode(INCREMENT_ONE_PIN, INPUT);
  pinMode(DECREMENT_ONE_PIN, INPUT);
  pinMode(INCREMENT_TEN_PIN, INPUT);
  pinMode(DECREMENT_TEN_PIN, INPUT);

  // Start the various components
  lcd.begin(LCD_NUM_COLS, LCD_NUM_ROWS);
  Wire.begin();
  cpuSerial.begin(CPU_BAUD_RATE);

  // Set the output voltage to zero to start
  writeDAC(MIN_DAC_NUMBER);

  // Read default flow from EEPROM
  float tempFlowRate = readEEPROMFloat(DF_ADDRESS);
  defaultFlowRate = int(tempFlowRate * FLOW_RATE_MULTIPLIER);
  targetFlowRate = defaultFlowRate;
  
  // Read and set the PID parameters from EEPROM
  Kp = readEEPROMFloat(KP_ADDRESS);
  Ki = readEEPROMFloat(KI_ADDRESS);
  Kd = readEEPROMFloat(KD_ADDRESS);
  flowPID.setKp(Kp);
  flowPID.setKi(Ki);
  flowPID.setKd(Kd);

  // Print some startup information to the user.
  // Naval Research Laboratory (NRL)
  // Pneumatically Modulated Liquid Delivery System (PMLDS)
  lcd.print("NRL-PMLDS v2.3.0");
  lcd.setCursor(0, 1);

  // Delay 3 seconds to give the flow meter time to warm up.
  delay(3000);

  lcd.print("Connecting");

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
  incrementOneState = digitalRead(INCREMENT_ONE_PIN);  
  decrementOneState = digitalRead(DECREMENT_ONE_PIN);
  incrementTenState = digitalRead(INCREMENT_TEN_PIN);
  decrementTenState = digitalRead(DECREMENT_TEN_PIN);

  if ( incrementOneState != prevIncrementOneState )
  {
    if ( !incrementOneState )
    {
      targetFlowRate = targetFlowRate + (ONE_INCREMENT * FLOW_RATE_MULTIPLIER);
    }

    prevIncrementOneState = incrementOneState;
  }

  if ( incrementTenState != prevIncrementTenState )
  {
    if ( !incrementTenState )
    {
      targetFlowRate = targetFlowRate + (TEN_INCREMENT * FLOW_RATE_MULTIPLIER);
    }

    prevIncrementTenState = incrementTenState;
  }

  if ( decrementOneState != prevDecrementOneState )
  {
    if ( !decrementOneState )
    {
      targetFlowRate = targetFlowRate - (ONE_INCREMENT * FLOW_RATE_MULTIPLIER); 
    }

    prevDecrementOneState = decrementOneState;
  }

  if ( decrementTenState != prevDecrementTenState )
  {
    if ( !decrementTenState )
    {
      targetFlowRate = targetFlowRate - (TEN_INCREMENT * FLOW_RATE_MULTIPLIER); 
    }

    prevDecrementTenState = decrementTenState;
  }

  if ( targetFlowRate > (MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    targetFlowRate = MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER;
  }

  if ( targetFlowRate < (MIN_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    targetFlowRate = MIN_FLOW_RATE * FLOW_RATE_MULTIPLIER; 
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
      // The flow rate is in uL/min but to save space and prevent excessive
      // floating-point calculations relative to the rolling average, the
      // instant flow rate and average flow rate are saved internally as
      // integers with uL/min x 10 (or the FLOW_RATE_MULTIPLIER). Make sure
      // to bound the flow rate as well.
      instantFlowRate = int(flowMeter.getFlowRate() * FLOW_RATE_MULTIPLIER);
      instantFlowRate = boundFlowRate(instantFlowRate);

      // Get the average of the last FLOW_RATE_QUEUE_SIZE non-zero instant flow rate
      // measurements. Use the average flow rate to calculate the pressure via the
      // PID control. This reduces excessive pressure control changes due to noise
      // in the flow rate reading.
      averageFlowRate = getAverageFlowRate(instantFlowRate);

      updateLCDFlowRate(averageFlowRate, ACTUAL_FLOW_RATE_ROW);

      if ( pidActivated )
      {
        float pidTarget = float(targetFlowRate) / float(FLOW_RATE_MULTIPLIER);
        float pidActual = float(averageFlowRate) / float(FLOW_RATE_MULTIPLIER);
        dacNumber = flowPID.compute(pidTarget, pidActual);
        
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

      if ( command.startsWith("TF") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {         
          targetFlowRate = readSerialFlowRate(command);
        }
        else if ( command.charAt(2) == GET_COMMAND )
        {
          writeSerialFlowRate(targetFlowRate);
        }
      }
      else if ( command.startsWith("DF") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          // The default flow rate is stored as an integer times FLOW_RATE_MULTIPLIER (10)
          defaultFlowRate = readSerialFlowRate(command);
          
          // Convert the default flow rate to a float because it is stored in EEPROM as a float
          // NOT as an integer times the FLOW_RATE_MULTIPLIER.
          float tempFlowRate = float(defaultFlowRate) / float(FLOW_RATE_MULTIPLIER);
          writeEEPROMFloat(DF_ADDRESS, tempFlowRate);
        }  
        else if ( command.charAt(2) == GET_COMMAND )
        {
          writeSerialFlowRate(defaultFlowRate);  
        }
      }
      else if ( command.startsWith("AF") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          // Nothing, cannot set the average flow rate.  
        }  
        else if ( command.charAt(2) == GET_COMMAND )
        {
          writeSerialFlowRate(averageFlowRate);
        }
      }
      else if ( command.startsWith("IF") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          // Nothing, cannot set the instant flow rate.
        }
        else if ( command.charAt(2) == GET_COMMAND )
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
      else if ( command.startsWith("KP") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          Kp = getCommandValue(command);
          writeEEPROMFloat(KP_ADDRESS, Kp);
          flowPID.setKp(Kp);
        }
        else if ( command.charAt(2) == GET_COMMAND )
        {
          String KpString = floatToString(Kp);
          cpuSerial.println(KpString);
        }
      }
      else if ( command.startsWith("KI") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          Ki = getCommandValue(command);
          writeEEPROMFloat(KI_ADDRESS, Ki);
          flowPID.setKi(Ki);
        }
        else if ( command.charAt(2) == GET_COMMAND )
        {
          String KiString = floatToString(Ki);
          cpuSerial.println(KiString);
        }      
      }
      else if ( command.startsWith("KD") )
      {
        if ( command.charAt(2) == SET_COMMAND )
        {
          Kd = getCommandValue(command);
          writeEEPROMFloat(KD_ADDRESS, Kd);
          flowPID.setKd(Kd);
        }
        else if ( command.charAt(2) == GET_COMMAND )
        {
          String KdString = floatToString(Kd);
          cpuSerial.println(KdString);
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
  Reads a float value from the EEPROM. A float value is four bytes long.
  
  @param initialAddress The start address of the four bytes in EEPROM memory of the float value.
  @return The float value.
*/
float readEEPROMFloat(int initialAddress)
{
  int currentAddress = initialAddress;

  union 
  {
    byte b[NUM_BYTES_PER_FLOAT];
    float f;
  } eepromValue;
  
  for (int i = 0; i < NUM_BYTES_PER_FLOAT; i++ )
  {
    eepromValue.b[i] = EEPROM.read(currentAddress);
    currentAddress++;
  }
  
  return eepromValue.f;
}

/*
  Writes a float value to EEPROM. A float value is four bytes long. A write will only occur if
  the value currently in EEPROM is different than the value to write to EEPROM. A check is 
  conducted to minimize writes to EEPROM, which is limited by hardware to 100,000 cycles.
  
  @param intialAdress The start address of the four bytes in EEPROM memory of the float value.
  @param floatValue The value as a float to write to EEPROM.
*/
void writeEEPROMFloat(int initialAddress, float floatValue)
{
  int currentAddress = initialAddress;
  
  union 
  {
    byte b[NUM_BYTES_PER_FLOAT];
    float f;
  } eepromValue;
  
  eepromValue.f = floatValue;
  
  float currentValue = readEEPROMFloat(initialAddress);
  if ( currentValue != floatValue )
  {  
    for (int i = 0; i < NUM_BYTES_PER_FLOAT; i++ )
    {
      EEPROM.write(currentAddress, eepromValue.b[i]);
      currentAddress++;
    }
  }
}

/*
  Reads the serial buffer until the TERMINATION_CHAR is reached or timeout.
 Will return "ERROR 01" as the command if the timeout was reached before
 the termination char was read.
 
 @return The command from the serial buffer.
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
    command = "ERROR 01"; 
  }

  return command;
}

/*
  Read the voltage from the cpu serial buffer and set the DAC based on the voltage. The PID control
 must be paused.
 
 @param command The command read from the serial buffer.
 @return The digital-to-analog converter number, which will be between 0 and MAX_DAC_NUMBER (255).
 */
int readSerialVoltage(String command)
{
  if ( !pidActivated )
  {
    float voltage = getCommandValue(command);
    int dac = int((voltage / MAX_VOLTAGE) * MAX_DAC_NUMBER);

    dac = boundDACNumber(dac);

    writeDAC(dac);
    
    return dac;
  }
}

/*
  Read the pressure from the cpu serial buffer and set the DAC based on the pressure. The PID control
 must be paused.
 
 @param command The command read from the serial buffer.
 @return The digital-to-analog converter number, which will be between 0 and MAX_DAC_NUMBER (255).
 */
int readSerialPressure(String command)
{
  if ( !pidActivated )
  {
    float pressure = getCommandValue(command);
    int dac = int((pressure / MAX_PRESSURE) * MAX_DAC_NUMBER);

    dac = boundDACNumber(dac);

    writeDAC(dac);
    
    return dac;
  } 
}

/*
  Reads a flow rate from the serial communication with the computer.
  
  @param command The command read from the serial buffer.
  @return The flow rate in uL/min * FLOW_RATE_MULTIPLIER (10).
 */
int readSerialFlowRate(String command)
{
  int flowRate = int(getCommandValue(command) * FLOW_RATE_MULTIPLIER);

  flowRate = boundFlowRate(flowRate);

  return flowRate;
}

/*
  Gets the value to set a parameter from the command.
  
  @param command The command read from the serial buffer.
  @return The value from a set command.
*/
float getCommandValue(String command)
{
  unsigned int commandLength = command.length();
  unsigned int bufferLength = commandLength + 1;
  char buffer[bufferLength];
  
  // All commands for setting a value have A=#### format, where "A" is replaced
  // with one or two letters. To obtain just the number portion, we use a 
  // substring starting at a position after the SET_COMMAND symbol (=), the first 
  // digit of the number and convert it to a char array for use in the atof() function.
  int commandValueIndex = 0;
  while ( commandValueIndex < commandLength )
  {
    if ( command.charAt(commandValueIndex) == SET_COMMAND )
    {
      // We need to increment to the start of the value, if we omit this
      // increment, it is the value plus the SET_COMMAND symbol (=)
      commandValueIndex++;
      break;  
    }
    
    commandValueIndex++;
  }
  
  String commandValue = command.substring(commandValueIndex); 
  commandValue.toCharArray(buffer, bufferLength);
  
  float value = atof(buffer);
  
  return value;
}

/*
  Limits the dac number to the minimum and maximum values. If the number is below MIN_DAC_NUMBER,
  the number is set to MIN_DAC_NUMBER. If the number is greater than MAX_DAC_NUMBER, then it is
  set to MAX_DAC_NUMBER.
  
  @param dac The digital-to-analog number.
  @return The digital-to-analog number bounded between MIN_DAC_NUMBER and MAX_DAC_NUMBER.
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
  Limits the flow rate to the minimum and maximum values. If the flow rate is greater than 
  MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER, then it is set to MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER.
  If the flow rate is less than the MIN_FLOW_RATE * FLOW_RATE_MULTIPLIER, then it is set to
  MIN_FLOW_RATE * FLOW_RATE_MULTIPLIER.
  
  @param flowRate The flow rate in uL/min * FLOW_RATE_MULTIPLIER.
  @return The flow rate in uL/min * FLOW_RATE_MULTIPLIER between the minimum and maximum flow rate * FLOW_RATE_MULTIPLIER.
*/
int boundFlowRate(int flowRate)
{
  if ( flowRate > (MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    flowRate = MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER;
  }
  
  if ( flowRate < (MIN_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    flowRate = 0;  
  }
  
  return flowRate;
}

/*
  Prints a flow rate to the serial communication with the computer.
  
  @param flowRate The flow rate in uL/min * FLOW_RATE_MULTIPLIER.
 */
void writeSerialFlowRate(int flowRate)
{ 
  float serialFlowRate = float(flowRate) / float(FLOW_RATE_MULTIPLIER);
  
  cpuSerial.print(serialFlowRate, FLOW_RATE_PRECISION);
  cpuSerial.println();
}

/*
  Prints the voltage to the serial communication with the computer.
  
  @param dacNumber The digital-to-analog number.
 */
void writeSerialVoltage(int dacNumber)
{
  float voltage = convertDACtoVoltage(dacNumber);

  cpuSerial.print(voltage, VOLTAGE_PRECISION);
  cpuSerial.println();
}

/*
  Prints the pressure in PSI to the serial communication with the computer.
  
  @param dacNumber The digital-to-analog number.
 */
void writeSerialPressure(int dacNumber)
{
  float pressure = convertDACtoPressure(dacNumber);

  cpuSerial.print(pressure, PRESSURE_PRECISION);
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
  
  @param dac The digital-to-analog number.
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
  
  @param flowRate The flow rate in uL/min * FLOW_RATE_MULTIPLIER.
  @param row The row on the LCD screen to print the flow rate, use either TARGET_FLOW_RATE_ROW or ACTUAL_FLOW_RATE_ROW.
 */
void updateLCDFlowRate(int flowRate, int row)
{
  // The number of decimal places to show. The precision is 1 if
  // the flow rate is between 10 and 99.9 uL/min and 0 if it is
  // greater than or equal to 100 uL/min.
  int precision = FLOW_RATE_PRECISION;
  
  lcd.setCursor(4, row);

  // If the flow rate is less than 10 uL/min, print a space
  if ( flowRate < 10.0 )
  {
    lcd.print(" ");
  }
  
  // If the flow rate is greater than or equal to 100 uL/min, print an
  // extra space. Remember the flow rate is saved as 10x.
  if ( flowRate >= (100 * FLOW_RATE_MULTIPLIER) && flowRate < (MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    lcd.print(" ");
    precision = 0;
  }
  else if ( flowRate >= (MAX_FLOW_RATE * FLOW_RATE_MULTIPLIER) )
  {
    precision = 0;  
  }

  float displayFlowRate = float(flowRate) / float(FLOW_RATE_MULTIPLIER);
  lcd.print(displayFlowRate, precision);
}

/*
  Updates the voltage on the LCD.
  
  @param dacNumber The digital-to-analog number.
 */
void updateLCDVoltage(int dacNumber)
{
  float voltage = convertDACtoVoltage(dacNumber);

  lcd.setCursor(12, 0);
  lcd.print(voltage, VOLTAGE_PRECISION);
}

/*
  Updates the pressure on the LCD.
  
  @param dacNumber The digital-to-analog number.
 */
void updateLCDPressure(int dacNumber)
{
  float pressure = convertDACtoPressure(dacNumber);

  lcd.setCursor(12, 1);

  if ( pressure < 10.0 )
  {
    lcd.print(" ");
  }

  lcd.print(pressure, PRESSURE_PRECISION);
}

/*
  Converts a dac number to a voltage (V).
  
  @param dac The digital-to-analog number.
 */
float convertDACtoVoltage(int dac)
{
  return (float(dac) / float(MAX_DAC_NUMBER)) * MAX_VOLTAGE;
}

/*
  Converts a dac number to pressure (PSI).
  
  @param dac The digital-to-analog number.
 */
float convertDACtoPressure(int dac)
{
  return (float(dac) / float(MAX_DAC_NUMBER)) * MAX_PRESSURE;
}

/*
  Adds the flow rate to the queue and averages the values in the flow rate queue.
    
  @param flowRate The flow rate in uL/min * FLOW_RATE_MULTIPLIER.
  @return The rolling average flow rate in uL/min * FLOW_RATE_MULTIPLIER.
*/
int getAverageFlowRate(int flowRate)
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

  int sum = 0;   
  int N = 0;
  for ( int i = 0; i < FLOW_RATE_QUEUE_SIZE; i++ )
  {
    if ( flowRateQueue[i] != 0.0 )
    {
      sum = sum + flowRateQueue[i];
      N++;
    }  
  }

  if ( N == 0 )
  {
    N = 1;  
  }

  int average = sum / N;

  return average;
}

/*
  Converts a float value to an ASCII string in scientific notation using a captial "E" for
  denoting the exponent.
  
  @param value The float value.
  @return The ASCII string representation of the float value in scientific notation.
*/
String floatToString(float value)
{ 
  String strValue;
  
  // Use the absolute value of the float but remember it was negative to begin with
  if ( value < 0 )
  {
    value = value * -1.0;  
    strValue = "-";
  }
  
  // Determin the exponent
  int exponent = 0;
  if ( value >= 10 )
  {
    while ( value >= 10 )
    {
      value = value / 10;
      exponent++;
    } 
  }
  else if ( (value > 0) && (value < 1) )
  {
    while ( value < 1 )
    {
      value = value * 10;
      exponent--;  
    }
  }
  
  // Convert to integers the left and right of the decimal.
  int leftOfDecimal = int(value);
  int rightOfDecimal = int(value * 100);
  rightOfDecimal = rightOfDecimal % 100;
  
  strValue = strValue + String(leftOfDecimal) + "." + String(rightOfDecimal) + "E" + String(exponent);
  
  return strValue;
}
