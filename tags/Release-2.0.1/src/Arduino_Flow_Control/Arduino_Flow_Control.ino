/*
 Created By: Christopher R. Field <christopher.field@nrl.navy.mil>
 Verison: 2.0.1
 
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
 
 The flow rates, both target and actual, are stored as integers with the ones position being the first
 digit right of the decimal with only one decimal of precision. The voltage is stored as a 100x integer
 with first two digits being the two digits right of the decimal. The pressure is stored as a 10x integer
 with only one decimal of precision. To get the human readable flow rates, divide by ten. To get the 
 human readable voltage, divide by 100, and to get the human readable pressure, divide by ten.
 
 The pressure is not actively monitored by the microcontroller, instead it is calculated from the control
 voltage based on the full range maximum of the EPC unit.
 
 The EPC unit can only control the pressure to 10% its maximum value. Any pressure below the 10% of its
 full range is consider 0 PSI.
 
 The flow rate is measured every second and the control voltage is calculated every two seconds.
 
 Verision 2.0.0 adds computer communication to the PMLDS system. The computer communicates with the
 PMLDS via the RS-232 protocol on a Serial port at 9600 Baud Rate, 8 data bits, 1 stop bit, no parity,
 and no hardware flow control. The commands are listed below:
 
 T=##.# Sets the target flow in uL/min
 T?     Gets the target flow rate in uL/min
 A?     Gets the actual flow rate in uL/min
 V=#.## Sets the EPC control voltage in volts, note that PID control must be paused
 V?     Gets the EPC control voltage in volts
 P=##.# Sets the pressure in PSI, note that PID control must be paused
 P?     Gets the pressure in PSI
 ||     Pause PID Control
 |>     Resume PID Control
 >>     Purge
 
 All commands must be terminated with a newline character (\n).
 
 Verision 2.0.1 is modified to work with the original, wire-wrap, prototype PMLDS box, which is
 still working. The increment/decrement buttons are different on the prototype PMLDS box.
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
const int MAX_VOLTAGE = 5; // volts
const int MAX_PRESSURE = 15; // psi
const long PURGE_TIME = 300000; // milliseconds (5 min)
const long MAX_PURGE_TIME = 240000; // milliseconds (4 min)

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
const int INCREMENT_BUTTON_ONE = 11;
const int DECREMENT_BUTTON_ONE = 10;
const int INCREMENT_BUTTON_TEN = 9;
const int DECREMENT_BUTTON_TEN = 8;

/*
  Flow sensor constants. Note the values 10x the actual values in uL/min.
 */
const int MAX_CONNECTION_ATTEMPTS = 3;
const int MAX_FLOW_RATE = 10000; // uL/min * 10
const int MIN_FLOW_RATE = 100; // uL/min * 10
const int DEFAULT_FLOW_RATE = 500; // uL/min * 10

/*
  PID constants.
 */
const float K_P = 0.0001; // 0.0005
const float K_I = 0.0005; // 0.001
const float K_D = 0.1; // 0.01

/*
  Timing constants.
 */
const long PURGE_DELAY = 1000; // milliseconds
const long PID_DELAY = 2500; // milliseconds
const long CPU_DELAY = 500; // milliseconds

/*
  CPU Serial Communication constants.
 */
const int RX_PIN = 12;
const int TX_PIN = 13;
const int CPU_BAUD_RATE = 9600;
const int MAX_COMMAND_LENGTH = 6;
const char GET_COMMAND = '?';
const char SET_COMMAND = '=';
const int ASCII_ZERO = 48;

int targetFlowRate = DEFAULT_FLOW_RATE; // uL/min * 10
int actualFlowRate = MIN_FLOW_RATE; // ul/min * 10
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
unsigned long lastCPUUpdate = 0; // milliseconds
unsigned long purgeStart = 0; // milliseconds
unsigned long currentTime = 0; // milliseconds
unsigned long updateDelay = PURGE_DELAY; // milliseconds

boolean purgeActivated = true;
boolean pidActivated = true;
boolean flowMeterConnected = false;

char command[MAX_COMMAND_LENGTH]; // The cpu serial command

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ASL1600 flowMeter;
PID flowPID(K_P, K_I, K_D, MAX_DAC_NUMBER, MIN_DAC_NUMBER);
SoftwareSerial cpuSerial(RX_PIN, TX_PIN);

void setup()
{ 
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
  lcd.print("NRL-PMLDS v2.0.1");
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
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
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
      targetFlowRate = targetFlowRate + 10;
    }

    prevIncrementOneState = incrementOneState;
  }

  if ( incrementTenState != prevIncrementTenState )
  {
    if ( !incrementTenState )
    {
      targetFlowRate = targetFlowRate + 100;
    }

    prevIncrementTenState = incrementTenState;
  }

  if ( decrementOneState != prevDecrementOneState )
  {
    if ( !decrementOneState )
    {
      targetFlowRate = targetFlowRate - 10; 
    }

    prevDecrementOneState = decrementOneState;
  }

  if ( decrementTenState != prevDecrementTenState )
  {
    if ( !decrementTenState )
    {
      targetFlowRate = targetFlowRate - 100; 
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

    if ( pidActivated || purgeActivated )
    {
      updateScreenFlowRate(targetFlowRate, TARGET_FLOW_RATE_ROW);
    }

    currentTime = millis();

    // Update only after a small delay, we do not need to be updating
    // every cycle.
    if ( (currentTime - lastUpdate) > updateDelay )
    {
      actualFlowRate = flowMeter.getFlowRate();

      if ( actualFlowRate < MIN_FLOW_RATE )
      {
        actualFlowRate = 0;
      }

      if ( actualFlowRate > MAX_FLOW_RATE )
      {
        actualFlowRate = MAX_FLOW_RATE; 
      }

      updateScreenFlowRate(actualFlowRate, ACTUAL_FLOW_RATE_ROW);

      if ( purgeActivated )
      {
        if ( (currentTime - purgeStart) > PURGE_TIME )
        {
          startPID();
        }
        else
        {
          updateScreenPurgeTime(currentTime, purgeStart);  
        }

        if ( (currentTime - purgeStart) > MAX_PURGE_TIME && dacNumber == MAX_DAC_NUMBER )
        {
          dacNumber = MAX_DAC_NUMBER / 2;
        }
      }

      if ( pidActivated )
      {
        float pidTarget = float(targetFlowRate) / 10.0;
        float pidActual = float(actualFlowRate) / 10.0;
        dacNumber = flowPID.compute(pidTarget, pidActual);
      }

      writeDAC(dacNumber);        

      // Update the screen voltage and pressure for every mode except purging.
      if ( !purgeActivated )
      {
        updateScreenVoltage(dacNumber);
        updateScreenPressure(dacNumber);
      }

      lastUpdate = millis();
    }

    // Handle the serial communication with the computer.
    if ( cpuSerial.available() )
    {
      readSerialCommand();

      if ( command[0] == 'T' )
      {
        if ( command[1] == SET_COMMAND )
        {         
          targetFlowRate = readSerialTargetFlowRate();
        }
        else if ( command[1] == GET_COMMAND )
        {
          writeSerialFlowRate(targetFlowRate);
        }
      }
      else if ( command[0] == 'A' )
      {
        if ( command[1] == SET_COMMAND )
        {
          // Nothing, cannot set the actual flow rate.  
        }  
        else if ( command[1] == GET_COMMAND )
        {
          writeSerialFlowRate(actualFlowRate);
        }
      }
      else if ( command[0] == 'V' )
      {
        if ( command[1] == SET_COMMAND )
        {
          dacNumber = readSerialVoltage();
        }
        else if ( command[1] == GET_COMMAND )
        {
          writeSerialVoltage(dacNumber);
        }
      }
      else if ( command[0] == 'P' )
      {
        if ( command[1] == SET_COMMAND )
        {
          dacNumber = readSerialPressure();
        }
        else if ( command[1] == GET_COMMAND )
        {
          writeSerialPressure(dacNumber);
        }
      }
      else if ( command[0] == '|' )
      {
        if ( command[1] == '|' )
        {
          pausePID();
        }
        else if ( command[1] == '>' )
        {
          startPID();
        }
      }
      else if ( command[0] == '>' )
      {
        if ( command[1] == '>' )
        {
          startPurge();
        } 
      }
    }
  }
}

/*
  Reads the serial buffer for a command.
 */
void readSerialCommand()
{
  int readIndex = 0;
  while ( cpuSerial.available() && readIndex < MAX_COMMAND_LENGTH )
  {
    command[readIndex] = cpuSerial.read();
    delay(10);
    readIndex++;    
  }
}

/*
  Read the voltage from the cpu serial buffer and set the DAC based on the voltage. The PID control
 must be paused.
 */
int readSerialVoltage()
{
  if ( !pidActivated )
  {
    int firstDigit = int(command[2]);
    int secondDigit = int(command[4]);
    int thirdDigit = int(command[5]);
    int readVoltage = ((firstDigit - ASCII_ZERO) * 100) + ((secondDigit - ASCII_ZERO) * 10) + (thirdDigit - ASCII_ZERO);

    int voltageDAC = int(((float(readVoltage) / 100.0) / float(MAX_VOLTAGE)) * float(MAX_DAC_NUMBER));

    voltageDAC = boundDACNumber(voltageDAC);

    return voltageDAC;
  }
}

/*
  Read the pressure from the cpu serial buffer and set the DAC based on the pressure. The PID control
 must be paused.
 */
int readSerialPressure()
{
  if ( !pidActivated )
  {
    int firstDigit = int(command[2]);
    int secondDigit = int(command[3]);
    int thirdDigit = int(command[5]);
    int readPressure = ((firstDigit - ASCII_ZERO) * 100) + ((secondDigit - ASCII_ZERO) * 10) + (thirdDigit - ASCII_ZERO);

    int pressureDAC = int(((float(readPressure) / 10.0) / float(MAX_PRESSURE)) * float(MAX_DAC_NUMBER));

    pressureDAC = boundDACNumber(pressureDAC);

    return pressureDAC;
  } 
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
  Read target flow rate from the serial communication with the computer.
 */
int readSerialTargetFlowRate()
{
  int firstDigit = int(command[2]);
  int secondDigit = int(command[3]);
  int thirdDigit = int(command[5]);
  int flowRate = ((firstDigit - ASCII_ZERO) * 100) + ((secondDigit - ASCII_ZERO) * 10) + (thirdDigit - ASCII_ZERO); 

  return flowRate;
}

/*
  Prints a flow rate to the serial communication with the computer.
 */
void writeSerialFlowRate(int flowRate)
{
  int leftOfDecimal = flowRate / 10;
  int rightOfDecimal = flowRate % 10;

  cpuSerial.print(leftOfDecimal);
  cpuSerial.print(".");
  cpuSerial.println(rightOfDecimal);
}

/*
  Prints the voltage to the serial communication with the computer.
 */
void writeSerialVoltage(int dacNumber)
{
  int voltage = convertDACtoVoltage(dacNumber);

  int leftOfDecimal = voltage / 100;
  int rightOfDecimal = voltage % 100;

  cpuSerial.print(leftOfDecimal);
  cpuSerial.print(".");
  cpuSerial.print(rightOfDecimal);

  if ( rightOfDecimal == 0 )
  {
    cpuSerial.print("0");  
  }

  cpuSerial.println();
}

/*
  Prints the pressure in PSI to the serial communication with the computer.
 */
void writeSerialPressure(int dacNumber)
{
  int pressure = convertDACtoPressure(dacNumber);

  int leftOfDecimal = pressure / 10;
  int rightOfDecimal = pressure % 10;

  cpuSerial.print(leftOfDecimal);
  cpuSerial.print(".");
  cpuSerial.println(rightOfDecimal);
}

/*
  Starts purging the lines and pump.
 */
void startPurge()
{
  purgeActivated = true;
  pidActivated = false;

  establishPurgeScreen();
  purgeStart = millis();
  dacNumber = MAX_DAC_NUMBER;
  writeDAC(dacNumber);

  updateDelay = PURGE_DELAY;
}

/*
  Pauses the PID control. This keeps the pressure/control voltage constant.
 */
void pausePID()
{
  pidActivated = false;
  purgeActivated = false;

  establishPauseScreen();

  updateDelay = PID_DELAY;
}

/*
  Starts the PID control scheme.
 */
void startPID()
{
  purgeActivated = false;
  pidActivated = true;

  establishPIDScreen();
  flowPID.restart();
  
  updateDelay = PID_DELAY;
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
  wipeScreen();

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
 | T | F | : |   | 3 | 0 | . | 0 |   | P | u | r | g | i | n | g | = Row 1
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | A | F | : |   | 3 | 0 | . | 0 |   |   |   | M | M | : | S | S | = Row 2
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15| = Col 
 
 The 'M' and 'S' are for minutes and seconds count priming timer.
 */
void establishPurgeScreen()
{
  wipeScreen();

  lcd.print("TF:");
  lcd.setCursor(9, 0);
  lcd.print("Purging");
  lcd.setCursor(0 , 1);
  lcd.print("AF:");
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
  wipeScreen();

  lcd.print("Paused");
  lcd.setCursor(9, 0);
  lcd.print("V:");
  lcd.setCursor(0 , 1);
  lcd.print("AF:");
  lcd.setCursor(9, 1);
  lcd.print("P: ");
}

/*
  Wipes the screen by printing 16 spaces on each row of the LCD screen.
 */
void wipeScreen()
{
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);  
}

/*
  Updates the count down timer related to purging the pump.
 */
void updateScreenPurgeTime(long time, long startTime)
{
  long msecRemaining = PURGE_TIME - (time - startTime);
  long secRemaining = msecRemaining / 1000;
  long minRemaining = secRemaining / 60;
  secRemaining = secRemaining % 60;

  lcd.setCursor(11, 1);

  if ( minRemaining < 10 )
  {
    lcd.print("0");
  }

  lcd.print(minRemaining);
  lcd.print(":");

  if ( secRemaining < 10 )
  {
    lcd.print("0");
  }

  lcd.print(secRemaining);
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
void updateScreenFlowRate(int flowRate, int row)
{
  lcd.setCursor(4, row);

  // If the flow rate is less than 10 uL/min, print a space, remember
  // the flow rate is stored as 10x the real flow rate.
  if ( flowRate < 100 )
  {
    lcd.print(" ");
  }

  // If the flow rate is greater than or equal to 100 uL/min, print an
  // extra space, remember the flow rate is stored as 10x the real
  // flow rate.
  if ( flowRate >= 1000 && flowRate < MAX_FLOW_RATE )
  {
    lcd.print(" ");
  }

  int leftOfDecimal = flowRate / 10;
  lcd.print(leftOfDecimal);

  // If the flow rate is than 100 uL/min, print a decimal place and
  // one digit of precision. Remember the flow rate is stored as
  // 10x the real flow rate. 
  if ( flowRate < 1000 )
  {
    int rightOfDecimal = flowRate % 10;
    lcd.print(".");
    lcd.print(rightOfDecimal);    
  }
}

/*
  Converts a dac number to a voltage (V * 100);
 */
int convertDACtoVoltage(int dac)
{
  return int((float(dac * 10) / float(MAX_DAC_NUMBER)) * float(MAX_VOLTAGE) * 10.0);
}

/*
  Converts a dac number to pressure (PSI * 10).
 */
int convertDACtoPressure(int dac)
{
  return int((float(dac * 10) / float(MAX_DAC_NUMBER)) * float(MAX_PRESSURE));
}

/*
  Update the voltage on the LCD.
 */
void updateScreenVoltage(int dacNumber)
{
  int voltage = convertDACtoVoltage(dacNumber);

  lcd.setCursor(12, 0);

  int leftOfDecimal = voltage / 100;
  int rightOfDecimal = voltage % 100;

  lcd.print(leftOfDecimal);
  lcd.print(".");
  lcd.print(rightOfDecimal);

  if ( rightOfDecimal == 0 )
  {
    lcd.print("0");  
  }
}

/*
  Update the pressure on the LCD.
 */
void updateScreenPressure(int dacNumber)
{
  int pressure = convertDACtoPressure(dacNumber);

  lcd.setCursor(12, 1);

  if ( pressure < 100 )
  {
    lcd.print(" ");
  }

  int leftOfDecimal = pressure / 10;
  int rightOfDecimal = pressure % 10;

  lcd.print(leftOfDecimal);
  lcd.print(".");
  lcd.print(rightOfDecimal);
}











