/*
 Created By: Christopher R. Field <christopher.field@nrl.navy.mil>
 Verison: 1.1.2
 
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
 
 Version 1.1.2 is designed to work with a Printed Circuit Board (PCB) that mounts directly on top of the
 Arduino Uno as a shield and fixes an initialization issue with the controller where the pump must be
 primed prior to regulating the flow. This version adds a 5 minute priming stage with maximum pressure 
 for 4 min (15 PSI) and half max pressure (7.5 PSI) for 1 minutes to prime the lines and purge any air from 
 the system. There is a problem with starting the system where the controller will open the EPC to full 
 scale and the flow meter will read 0 uL/min because off scale on the flow meter is 0 uL/min; thus, the 
 controller never throttles back the EPC pressure to regulate the flow even though liquid is still be 
 pumped through the system. The 3 minute priming stage allows for bubbles and air to purged from the 
 system and generate a stable liquid flow. Once a stable liquid flow is established, the microcontroller 
 can begin PID control to the target flow rate.
 */
#include <LiquidCrystal.h>
#include <Wire.h>
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
const long PRIME_PUMP_TIME = 300000; // milliseconds
const long MAX_PRIME_PUMP_TIME = 240000; // milliseconds

/*
  LCD pins
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

/*
  One's and ten's place increment/decrement switches pins.
 */
const int INCREMENT_BUTTON_ONE = 8;
const int DECREMENT_BUTTON_ONE = 9;
const int INCREMENT_BUTTON_TEN = 10;
const int DECREMENT_BUTTON_TEN = 11;

/*
  Flow sensor constants. Note the values 10x the actual values in uL/min.
 */
const int MAX_CONNECTION_ATTEMPTS = 3;
const int MAX_FLOW_RATE = 1000; // uL/min * 10
const int MIN_FLOW_RATE = 100; // uL/min * 10
const int DEFAULT_FLOW_RATE = 300; // uL/min * 10

/*
  PID constants.
 */
const float K_P = 0.02;
const float K_I = 0.001;
const float K_D = 0.01;

const long PRIME_PUMP_DELAY = 1000; // milliseconds
const long PID_DELAY = 2000; // milliseconds

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
long lastUpdate = 0; // milliseconds
long primePumpStart = 0; // milliseconds
long currentTime = 0; // milliseconds
long updateDelay = PRIME_PUMP_DELAY; // milliseconds

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ASL1600 flowMeter;
PID flowPID(K_P, K_I, K_D, MAX_DAC_NUMBER, MIN_DAC_NUMBER);

void setup()
{ 
  pinMode(INCREMENT_BUTTON_ONE, INPUT);
  pinMode(DECREMENT_BUTTON_ONE, INPUT);
  pinMode(INCREMENT_BUTTON_TEN, INPUT);
  pinMode(DECREMENT_BUTTON_TEN, INPUT);

  lcd.begin(LCD_NUM_COLS, LCD_NUM_ROWS);
  Wire.begin();

  // Set the output voltage to zero to start
  writeDAC(MIN_DAC_NUMBER);

  // Print some startup information to the user.
  // Naval Research Laboratory (NRL)
  // Pneumatically Modulated Liquid Delivery System (PMLDS)
  lcd.print("NRL-PMLDS v1.1.2");
  lcd.setCursor(0, 1);
  lcd.print("Connecting");

  /*
   If the flow meter and Arduino are turned on at the same time, the Arduino will check for connection
   before the flow meter has warmed up, resulting in not a connection. The Arduino could be reset, but
   instead check for connectivity with the flow meter three times before concluding a disconnect.
   */
  int attempts = 1;
  int flowFactor = -1;
  while ( attempts <= MAX_CONNECTION_ATTEMPTS )
  {
    flowMeter.begin();
    flowFactor = flowMeter.getFlowFactor();

    if ( flowFactor == -1 )
    {
      // Print a period for each attempt
      lcd.print(".");
      attempts++;

      // Wait a little bit before attempting another connection check.
      delay(2000);
    }
    else
    {
      // Connection established, exit the loop.
      break;
    }
  }

  lcd.setCursor(0, 1);

  if ( flowFactor == -1 )
  {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Flow Meter");
    lcd.setCursor(0, 1);
    lcd.print("Not Connected");
  }
  else
  {
    // Report the flow factor as an indicator it is working
    lcd.print("Flow Factor: ");
    lcd.print(flowFactor);

    delay(3000);

    // Start priming the pump
    establishPrimingScreen();
    primePumpStart = millis();
    dacNumber = MAX_DAC_NUMBER;
    writeDAC(dacNumber);
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

  // The read function must be called each iteration of the loop method.
  flowMeter.read();

  // Only update the LCD and pressure if the flow meter is connected
  // and reading a flow rate.
  if ( flowMeter.getFlowFactor() != -1 )
  { 
    updateFlowRate(targetFlowRate, TARGET_FLOW_RATE_ROW);

    currentTime = millis();

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

      updateFlowRate(actualFlowRate, ACTUAL_FLOW_RATE_ROW);

      if ( (currentTime - primePumpStart) > PRIME_PUMP_TIME )
      {
        float pidTarget = ((float)targetFlowRate / 10.0);
        float pidActual = ((float)actualFlowRate / 10.0);
        dacNumber = flowPID.compute(pidTarget, pidActual);

        writeDAC(dacNumber);

        if ( updateDelay == PRIME_PUMP_DELAY )
        {
          establishControlScreen();
        }

        updateVoltage(dacNumber);
        updatePressure(dacNumber);

        updateDelay = PID_DELAY;
      }
      else
      {
        // Change to half max pressure after one minute
        if ( (currentTime - primePumpStart) > MAX_PRIME_PUMP_TIME && dacNumber == MAX_DAC_NUMBER)
        {
          dacNumber = MAX_DAC_NUMBER / 2;
          writeDAC(dacNumber);        
        }
        
        updatePrimePumpTime(currentTime, primePumpStart);
        updateDelay = PRIME_PUMP_DELAY;
      }

      lastUpdate = millis();
    }
  }
}

/*
  Wipes the screen and prints the control screen. The control screen is as follows, where
 the vertical lines designate cells in the LCD screen and are not actually printed.
 
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+  
 Row 0:| T | F | : |   | 3 | 0 | . | 0 |   |   | V | : |   | 5 | . | 0 |
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 Row 1:| A | F | : |   | 3 | 0 | . | 0 |   |   | P | : | 1 | 5 | . | 0 |
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 Col   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15|
 */
void establishControlScreen()
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
 Row 0:| T | F | : |   | 3 | 0 | . | 0 |   | P | r | i | m | i | n | g |
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 Row 1:| A | F | : |   | 3 | 0 | . | 0 |   |   |   | M | M | : | S | S |
 +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ 
 Col   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10| 11| 12| 13| 14| 15|
 
 The 'M' and 'S' are for minutes and seconds count priming timer.
 */
void establishPrimingScreen()
{
  wipeScreen();

  lcd.print("TF:");
  lcd.setCursor(9, 0);
  lcd.print("Priming");
  lcd.setCursor(0 , 1);
  lcd.print("AF:");
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
  Updates the count down timer related to priming the pump.
 */
void updatePrimePumpTime(long time, long startTime)
{
  long msecRemaining = PRIME_PUMP_TIME - (time - startTime);
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
  Wire.send(COMMAND);
  Wire.send(dac);
  Wire.endTransmission();
}

/*
  Update a flow rate on the LCD.
 */
void updateFlowRate(int flowRate, int row)
{
  lcd.setCursor(4, row);

  if ( flowRate < MIN_FLOW_RATE )
  {
    lcd.print(" ");
  }

  if ( flowRate >= MAX_FLOW_RATE )
  {
    lcd.print(">");
    flowRate = MAX_FLOW_RATE;
  }

  int leftOfDecimal = flowRate / 10;
  lcd.print(leftOfDecimal);

  if ( flowRate < MAX_FLOW_RATE )
  {
    int rightOfDecimal = flowRate % 10;
    lcd.print(".");
    lcd.print(rightOfDecimal);    
  }
}

/*
  Update the voltage on the LCD.
 */
void updateVoltage(int dac)
{
  int voltage = ((dac * 100) / MAX_DAC_NUMBER) * MAX_VOLTAGE;

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
void updatePressure(int dac)
{
  int pressure = ((dac * 10) / MAX_DAC_NUMBER) * MAX_PRESSURE;

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
