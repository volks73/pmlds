#include <SoftwareSerial.h>
#include <Wire.h>
#include <ASL1600.h>

const int DAC_ADDRESS = 0x2D;
const byte COMMAND = 0;
const int MAX_DAC_NUMBER = 255;
const int RX_PIN = 12;
const int TX_PIN = 13;
const int CPU_BAUD_RATE = 9600;

SoftwareSerial cpuSerial(RX_PIN, TX_PIN);
ASL1600 flowMeter;

void setup()
{
  cpuSerial.begin(CPU_BAUD_RATE);

  Wire.begin();
  writeDAC(MAX_DAC_NUMBER);

  delay(3000);
  
  flowMeter.begin();
}

void loop()
{
  flowMeter.read();
  
  float flowRate = float(flowMeter.getFlowRate()) / 10.0;
  cpuSerial.print("flowRate: ");
  cpuSerial.print(flowRate, 1);
  cpuSerial.println();
  
  delay(1000);
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







