/*
  Created By: Christopher R. Field <christopher.field@nrl.navy.mil>
  Verison: 1.0.0
  Copyright 2011
  
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
  datasheets for each IC to determine the electrical connections.
  
  The flow rates, both target and actual, are stored as integers with the ones position being the first
  digit right of the decimal with only one decimal of precision. The voltage is stored as a 100x integer
  with first two digits being the two digits right of the decimal. The pressure is stored as a 10x integer
  with only one decimal of precision. To get the human readable flow rates, divide by ten. To get the 
  human readable voltage, divide by 100, and to get the human readable pressure, divide by ten.
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
const int COMMAND = 0;
const int MAX_DAC_NUMBER = 255;
const int MIN_DAC_NUMBER = 0;
const int MAX_VOLTAGE = 5;
const int MAX_PRESSURE = 15;

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
const int INCREMENT_BUTTON_ONE = 11;
const int DECREMENT_BUTTON_ONE = 10;
const int INCREMENT_BUTTON_TEN = 9;
const int DECREMENT_BUTTON_TEN = 8;

/*
  Flow sensor constants. Note the values 10x the actual values in uL/min.
*/
const int MAX_FLOW_RATE = 1000;
const int MIN_FLOW_RATE = 100;
const int DEFAULT_FLOW_RATE = 300;
const float K_P = 0.02;
const float K_I = 0.001;
const float K_D = 0.01;
const long READ_DELAY = 1000; // milliseconds
const long WRITE_DELAY = 2000; // milliseconds

int targetFlowRate = DEFAULT_FLOW_RATE;
int actualFlowRate = MIN_FLOW_RATE;
int dacNumber = 0;
int incrementOneState = HIGH;
int decrementOneState = HIGH;
int incrementTenState = HIGH;
int decrementTenState = HIGH;
int prevIncrementOneState = HIGH;
int prevDecrementOneState = HIGH;
int prevIncrementTenState = HIGH;
int prevDecrementTenState = HIGH;
long lastRead = 0;
long lastWrite = 0;

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ASL1600 flowMeter;
PID flowPID(K_P, K_I, K_D, MAX_DAC_NUMBER, MIN_DAC_NUMBER);

void setup()
{ 
  lcd.begin(LCD_NUM_COLS, LCD_NUM_ROWS);
  Wire.begin();
  
  // Set the output voltage to zero to start
  writeDAC(dacNumber);
  
  flowMeter.begin();
  
  // Print some startup information to the user.
  lcd.print("NRL-CANS v1.0.0");
  lcd.setCursor(0, 1);
  
  int flowFactor = flowMeter.getFlowFactor();
  
  if ( flowFactor == -1 )
  {
    lcd.print("Not Connected");
  }
  else
  {
    lcd.print("Flow Factor: ");
    lcd.print(flowFactor);
  }
  
  delay(3000);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  
  lcd.print("TF: ");
  lcd.setCursor(9, 0);
  lcd.print("V: ");
  lcd.setCursor(0, 1);
  lcd.print("AF: ");
  lcd.setCursor(9, 1);
  lcd.print("P: ");
    
  pinMode(INCREMENT_BUTTON_ONE, INPUT);
  pinMode(DECREMENT_BUTTON_ONE, INPUT);
  pinMode(INCREMENT_BUTTON_TEN, INPUT);
  pinMode(DECREMENT_BUTTON_TEN, INPUT);
}

void loop()
{
  // The read function must be called each iteration of the loop method.
  flowMeter.read();
  
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
  
  updateFlowRate(targetFlowRate, TARGET_FLOW_RATE_ROW);
  
  // Read the flow meter once a second
  if ( (millis() - lastRead) > READ_DELAY )
  {
    actualFlowRate = flowMeter.getFlowRate();
    
    if ( actualFlowRate < MIN_FLOW_RATE )
    {
      actualFlowRate = 0.0;
    }
        
    updateFlowRate(actualFlowRate, ACTUAL_FLOW_RATE_ROW);
    
    lastRead = millis();
  }

  // Update the EPC control voltage once every two seconds
  if ( (millis() - lastWrite) > WRITE_DELAY )
  { 
    float pidTarget = ((float)targetFlowRate / 10.0);
    float pidActual = ((float)actualFlowRate / 10.0);
    dacNumber = flowPID.compute(pidTarget, pidActual);

    writeDAC(dacNumber);
 
    updateVoltage(dacNumber);
    updatePressure(dacNumber);
    
    lastWrite = millis();
  }
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
  
  if ( flowRate < MIN_FLOW_RATE || flowRate >= MAX_FLOW_RATE )
  {
    lcd.print(" ");
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
