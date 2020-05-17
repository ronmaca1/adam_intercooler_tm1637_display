#include <SPI.h>
#include <SerialCommand.h>
#include <EEPROM.h>
#include <stdio.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define CLK 3
#define DIO 2

#define _DEBUG_
#define erased 0
#define eepromparamsize 2 // EEPROM storage allocation units in bytes == sizeof(int)
#define sigaddr 1         // first active eeprom addr. skipped addr 0.
#define setcalAddr (sigaddr + eepromparamsize)
#define setr1Addr (setcalAddr + eepromparamsize)
#define setcelorfahrAddr (setr1Addr + eepromparamsize)
#define setbrightAddr (setcelorfahrAddr + eepromparamsize)
// (use for MCP4901 single channel DAC)
// VREF gain 1(EXT. Vref)
#define MCP4901ACONFIGBITS 0x30

typedef unsigned int storedparamtype;
// <settable parameters>
storedparamtype calval;
storedparamtype brightval;
storedparamtype fahrenheit;
// which analog pin to connect
// the value of the 'other' resistor
// it's actually 10000 ohms, but fudged here
// so as to allow for calibration by the DAC feeding the resistor
storedparamtype seriesresistor;
// </end of settable parameters>

int readIndex = 0; // the index of the current reading
// The beta coefficient of the thermistor (usually 3000-4000)
int thermbeta = 3778;
//int thermbeta = 3600; // 3778 was calculated value
// resistance at tempnom (25c)
int thermnomres = 11150;
// temp. for nominal resistance (almost always 25 C)
int tempnom = 25;

const byte inputPin = A4;
const byte dacsel = 10;
const byte ldac = 9;
const byte reset_out = 12;
const int numReadings = 25;
float samples[numReadings]; // the samples from the analog input

byte dacoutL = 0;
byte dacoutH = 0;
float average = 0;
float total = 0;

void get_stored_params(void)
{
  EEPROM.get(setcalAddr, calval);
  EEPROM.get(setr1Addr, seriesresistor);
  EEPROM.get(setcelorfahrAddr, fahrenheit);
  EEPROM.get(setbrightAddr, brightval);
}

TM1637Display display(CLK, DIO);

SerialCommand sCmd;

void setup()
{
  digitalWrite(reset_out, HIGH);
  pinMode(reset_out, OUTPUT);
  digitalWrite(ldac, HIGH);
  pinMode(ldac, OUTPUT);
  digitalWrite(dacsel, HIGH);
  pinMode(dacsel, OUTPUT);

  Serial.begin(9600);

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.begin();
  // gets parameters stored in eeprom
  // loads them into settable parameters listed above
  get_stored_params();
  // send the calval level to the calibration DAC

  caldacset();
  display.clear();
  EEPROM.get(setbrightAddr, brightval);
  display.setBrightness(brightval);
  // kill the digital buffer on our analog input
  DIDR0 |= _BV(ADC4D);

  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("INIT", init_eeprom); //1
  sCmd.addCommand("SETCAL", set_cal);
  sCmd.addCommand("SETR1", set_seriesresistor);
  sCmd.addCommand("SETF", set_fahrenheit);
  sCmd.addCommand("SETC", set_celsius);
  sCmd.addCommand("SETBRIGHT", set_bright);
  sCmd.addCommand("HELP", show_help);
  sCmd.addCommand("PSET", print_current_settings);
  sCmd.addCommand("RESET", reset_system);
  sCmd.setDefaultHandler(unrecognized);

  // clear the sample array
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    samples[thisReading] = 0;
  }
}

void loop()
{
  // <check for incoming command>
  sCmd.readSerial();
  // low pass fir by damellis
  // subtract the last reading:
  total = total - samples[readIndex];
  // read from the sensor:
  samples[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + samples[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings)
  {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  average = total / numReadings;
  // end of low pass fir
#ifdef _DEBUG_
  // calculate the average:
  Serial.print("Average analog reading ");
  Serial.println(average);
#endif
  // convert the value to resistance
  average = 1023 / average - 1;
  average = seriesresistor / average;
#ifdef _DEBUG_
  Serial.print("Thermistor resistance ");
  Serial.println(average);
#endif
  // steinhart hart by adafruit
  float steinhart;
  steinhart = average / thermnomres;     // (R/Ro)
  steinhart = log(steinhart);            // ln(R/Ro)
  steinhart /= thermbeta;                // 1/B * ln(R/Ro)
  steinhart += 1.0 / (tempnom + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;           // Invert
  steinhart -= 273.15;                   // convert to C
                                         // end of steinhart hart
                                           

  if (fahrenheit == false) // output degrees in Centigrade
  {
   // display.showNumberDec (round((steinhart)), false);
    display.showNumberDecEx((steinhart)*10,0b00100000, false); 
  }
  if (fahrenheit == true) // output degrees in Fahrenheit
  {   
    //display.showNumberDec (round(((steinhart) * (9.0 / 5.0) + 32)), false);
    display.showNumberDecEx(((steinhart) * (9.0 / 5.0) + 32)*10,0b00100000, false);
  }
  delay(150); // delay in between reads for stability
}

// <helper functions for settings>
void caldacset(void)
{
  dacoutH = MCP4901ACONFIGBITS | ((calval >> 4) & 0x0F);
  dacoutL = (calval << 4) & 0xF0;
  digitalWrite(dacsel, LOW);
  delayMicroseconds(10); // let the DAC get ready
  SPI.transfer(dacoutH);
  SPI.transfer(dacoutL);
  delayMicroseconds(10); // let the DAC settle
  digitalWrite(dacsel, HIGH);
  digitalWrite(ldac, LOW);
  delayMicroseconds(10);
  digitalWrite(ldac, HIGH);
}
storedparamtype eepromreturn(unsigned int addr)
{
  storedparamtype eepromdata;
  EEPROM.get(addr, eepromdata);
  return eepromdata;
}

void erase_eeprom(void)
{
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.put(i, erased);
  }
}

void set_defaults(void)
{
  EEPROM.put(sigaddr, 0xAA55);
  EEPROM.put(setcalAddr, 251); //5 volts inital reference
  EEPROM.put(setr1Addr, 10000);
  EEPROM.put(setcelorfahrAddr, true); // display fahrenheit by default
  EEPROM.put(setbrightAddr, 3);       // set display at half brightness
}

void init_eeprom()
{
  String arg;
  arg = sCmd.next();

  if (arg != NULL)
  {
    if (arg == "+++")
    {
      erase_eeprom();
      set_defaults();
      delay(100);
      reset_system();
    }
    else
    {
      Serial.println("");
      Serial.println(F("OOPSIE. wrong password"));
      Serial.println("");
    }
  }
  else
  {
    Serial.println("");
    Serial.println(F("No secret code given, nothing has been done. Bye :-)"));
    Serial.println("");
    return;
  }
}

void set_cal()
{
  char *arg;
  int argintval;
  arg = sCmd.next();
  if (arg != NULL)
  {
    argintval = atoi(arg);
    if (argintval >= 200 && argintval <= 255) /////////////////////////////////////////
    {
      EEPROM.put(setcalAddr, (storedparamtype)(argintval));
      EEPROM.get(setcalAddr, calval);
      caldacset();
    }
    else
    {
      Serial.println(F(" setcal cannot be less than 200 or "));
      Serial.println(F(" more than 255. Nothing was changed"));
    }
  }
}

void set_seriesresistor(void)
{
  char *arg;
  uint16_t argintval;
  arg = sCmd.next();
  if (arg != NULL)
  {
    argintval = atoi(arg);
    if (argintval >= 9000 && argintval <= 12000)
    {
      EEPROM.put(setr1Addr, (storedparamtype)(argintval));
      EEPROM.get(setr1Addr, seriesresistor);
    }
    else
    {
      Serial.println(F(" setr1 cannot be less than 9000 or "));
      Serial.println(F(" more than 12000. Nothing was changed"));
    }
  }
}

void set_fahrenheit(void)
{
  EEPROM.put(setcelorfahrAddr, (storedparamtype)(true));
  EEPROM.get(setcelorfahrAddr, fahrenheit);
}
void set_celsius(void)
{
  EEPROM.put(setcelorfahrAddr, (storedparamtype)(false));
  EEPROM.get(setcelorfahrAddr, fahrenheit);
}

void set_bright(void)
{
  char *arg;

  arg = sCmd.next();
  if (arg != NULL)
  {
    brightval = atoi(arg);
    if (brightval >= 0 && brightval <= 7)
    {
      EEPROM.put(setbrightAddr, (storedparamtype)(brightval));
      EEPROM.get(setbrightAddr, brightval);
      display.setBrightness(brightval);
    }
    else
    {
      Serial.println(F(" setbright cannot be less than 0 or "));
      Serial.println(F(" more than 7. Nothing was changed"));
    }
  }
}

void show_help()
{
  Serial.println("");
  Serial.println(F("\"INIT +++\" sets the stored settings to default"));
  Serial.println("");

  Serial.println(F("\"PSET\" Shows current settings"));
  Serial.println("");

  Serial.println(F("\"SETCAL xxx\" sets the temperature accuracy. right around 251 is good"));
  Serial.println(F("xxx is limited to 190 low and 255 high"));
  Serial.println(F("values lower or higher we be clipped to the above"));

  Serial.println(F("\"SETF\" sets the temperature"));
  Serial.println(F(" display to Fahrenheit "));
  Serial.println("");

  Serial.println(F("\"SETC\" sets the temperature"));
  Serial.println(F(" display to Celsius "));
  Serial.println("");

  Serial.println(F("\"SETBRIGHT\" sets the display "));
  Serial.println(F(" brightnes in steps from  0 to 7 "));
  Serial.println(F("values lower or higher will be ignored"));
  Serial.println("");

  Serial.println(F("\"SETR1\" sets the seriesresistor "));
  Serial.println(F(" value from 9000 to 12000 ohms "));
  Serial.println(F("values lower or higher will be ignored"));
  Serial.println("");
}

void print_current_settings()
{
  Serial.println("");

  Serial.print(F("EEPROM signature is: "));
  Serial.println((eepromreturn(sigaddr)), HEX);
  Serial.println("");

  Serial.print(F("setcal is: "));
  Serial.println((eepromreturn(setcalAddr)), DEC);
  Serial.println("");

  Serial.print(F("setbright is: "));
  Serial.println((eepromreturn(setbrightAddr)), DEC);
  Serial.println("");

  Serial.print(F("seriesresistor is: "));
  Serial.println((eepromreturn(setr1Addr)), DEC);
  Serial.println("");

  Serial.print(F("setfahr  is: "));
  if (eepromreturn(setcelorfahrAddr) == false)
  {
    Serial.println(F("display temperature is in Centigrade"));
  }
  else if (eepromreturn(setcelorfahrAddr) == true)
  {
    Serial.println(F("display temperature is in Fahrenheit"));
  }

  else
  {
    Serial.println(F(" oops."));
    Serial.println("");
  }
}

void reset_system(void)
{
  digitalWrite(reset_out, LOW);
}

void unrecognized(const char *command)
{
  Serial.println("");
  Serial.println(F("Unknown command"));
  Serial.println("");
}