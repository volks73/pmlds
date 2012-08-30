#include <SoftwareSerial.h>
#include <Wire.h>

const int DAC_ADDRESS = 0x2D;
const byte COMMAND = 0;
const int READ_BUFFER_SIZE = 128;
const int SYNC_BUFFER_SIZE = 4;
const byte SYNC_CODE = 0x7F;
const int MAX_DAC_NUMBER = 255;
const int BAUD_RATE = 19200;
const int ASCII_ZERO = 48;
const int COMMAND_DELAY = 100;
const int RX_PIN = 12;
const int TX_PIN = 13;
const int CPU_BAUD_RATE = 9600;

byte syncBuffer[SYNC_BUFFER_SIZE];
int flowFactor = 29;
float instantFlowRate = 0.0;
float averageFlowRate = 0.0;
int numFlowRateReadings = 0;

SoftwareSerial cpuSerial(RX_PIN, TX_PIN);

void setup()
{
  cpuSerial.begin(CPU_BAUD_RATE);

  Wire.begin();
  writeDAC(MAX_DAC_NUMBER);

  for ( int i = 0; i < SYNC_BUFFER_SIZE; i++ )
  {
    syncBuffer[i] = 0x00;
  }

  printBuffer();

  delay(3000);

  Serial.begin(BAUD_RATE);

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("s");
  delay(COMMAND_DELAY);

  clearAllBytes();

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("pw=expand");
  delay(COMMAND_DELAY);

  clearAllBytes();

  if ( Serial.available() )
  {
    clearAllBytes(); 
  }

  Serial.print("raw=");
  Serial.print("0");
  Serial.print("\n");
  delay(COMMAND_DELAY);

  clearAllBytes();

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("mod=");
  Serial.print("F");
  Serial.print("\n");
  delay(COMMAND_DELAY);

  clearAllBytes();

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("res="); 
  Serial.print("7");
  Serial.print("\n");
  delay(COMMAND_DELAY);

  clearAllBytes();

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("info\n");
  delay(COMMAND_DELAY);

  clearNBytes(54);

  int firstDigit = Serial.read();
  int secondDigit = Serial.read();

  if ( firstDigit != -1 && secondDigit != -1 )
  {
    /*
     A simple method for converting two ASCII digits into an integer value. For example, if the first digit is ASCII DEC 50 (char 2) and 
     the second digit is ASCII DEC 57 (char 9), the integer value is 29. The numbers are listed in decimal order in ASCII, so simiply 
     subtract char zero (ASCII 48) from both digits and multiple the first digit by 10.
     */
    flowFactor = (firstDigit - ASCII_ZERO) * 10 + (secondDigit - ASCII_ZERO);
  }

  clearAllBytes();

  cpuSerial.print("flowFactor: ");
  cpuSerial.print(flowFactor);
  cpuSerial.println();

  if ( Serial.available() )
  {
    clearAllBytes();  
  }

  Serial.print("go\n");
  delay(COMMAND_DELAY);
}

void loop()
{
  int numBytes = Serial.available();

  cpuSerial.print("numBytes: ");
  cpuSerial.print(numBytes);
  cpuSerial.println();

  if ( numBytes )
  {
    averageFlowRate = 0;
    numFlowRateReadings = 0;

    for ( int i = 0; i < numBytes; i++ )
    {
      cpuSerial.print("i: ");
      cpuSerial.print(i);
      cpuSerial.println();

      for ( int j = 0; j < SYNC_BUFFER_SIZE; j++ )
      {
        if ( j+1 < SYNC_BUFFER_SIZE )
        {
          syncBuffer[j] = syncBuffer[j+1];
        }
        else
        {
          syncBuffer[j] = Serial.read();
        }
      }

      printBuffer();

      if ( syncBuffer[0] == SYNC_CODE && syncBuffer[1] == SYNC_CODE && syncBuffer[2] != SYNC_CODE )
      {
        int rawFlowRate = int(word(syncBuffer[2], syncBuffer[3]));
        cpuSerial.print("raw flow rate: ");
        cpuSerial.print(rawFlowRate);
        cpuSerial.println();

        instantFlowRate = float(rawFlowRate) / float(flowFactor);
        cpuSerial.print("instant flow rate: ");
        cpuSerial.print(instantFlowRate, 1);
        cpuSerial.println();
        cpuSerial.println();

        averageFlowRate = averageFlowRate + instantFlowRate;
        numFlowRateReadings++;
      }
    }

    averageFlowRate = averageFlowRate / float(numFlowRateReadings);
    cpuSerial.print("average flow rate: ");
    cpuSerial.print(averageFlowRate, 1);
    cpuSerial.println();
  }
  
  delay(1000);
}

void clearNBytes(int numBytes)
{
  for ( int i = 0; i < numBytes; i++ )
  {
    char readChar = Serial.read();
    cpuSerial.print("readChar[");
    cpuSerial.print(i);
    cpuSerial.print("]: ");
    cpuSerial.print(readChar);
    cpuSerial.println();
    delay(60);
  }
}


void clearAllBytes()
{
  int numChar = 0;
  while ( Serial.available() )
  {
    char readChar = Serial.read();
    cpuSerial.print("readChar[");
    cpuSerial.print(numChar);
    cpuSerial.print("]: ");
    cpuSerial.print(readChar);
    cpuSerial.println();
    delay(60);

    numChar++;
  }
}

void printBuffer()
{
  cpuSerial.println();

  for ( int i = 0; i < SYNC_BUFFER_SIZE; i++ )
  {
    cpuSerial.print("buffer[");
    cpuSerial.print(i);
    cpuSerial.print("]: ");
    cpuSerial.print(syncBuffer[i], HEX);
    cpuSerial.print(" int: ");
    cpuSerial.print(syncBuffer[i]);
    cpuSerial.println();
  }  

  cpuSerial.println();
}

void printNumBytes()
{
  int numBytes = Serial.available();
  cpuSerial.print("numBytes: ");
  cpuSerial.print(numBytes);
  cpuSerial.println(); 
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







