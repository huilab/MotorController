/* Motor Controller Version 1.6
 *  
 *  Author: Erik Werner 2016 10 19
 *  Co-Authors: Amrith Karunaratne 2016 10 19
 *  
 *  Last update: 2018 05 03 Erik Werner
 *
 * Revision History:
 * 1.0: Initial release
 * 1.1 Improved speed data printing read-ability
 *    Added option to reverse motor direction every 5 minutes
 * 1.2 Changed reverse algorithm to fix motor 3 stalling issues
 *    Changeed to suppress EEPROM write warnings
 * 1.3 Changed EEPROMEx updateInt to writeInt
 * 1.4 Fixed a bug where autoReverse flag was not saved and loaded on power cycle
 *    Added the ability to change the autoReverse duration
 * 1.5 Fixed a bug where negative signs were ignored when setting motor speeds
 *    Added additional notes to help text to display on processor reset 
 * 1.6 Added limit checks to motor speed and auto reverse interval input
 *    Improved readability of statistics text output
 *
 * Firmware for Arduino Uno and Sain Smart Motor Shield to Drive 4 DC motors
 * Note: Sain Smart Motor Shield uses Adafruit Motor Library v1.0
 * Note: EEPROM function uses EEPROMEx Library
*/

#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <AFMotor.h>

//configure code for 4 motors
#define N_MOTORS 4
//supress warnings if EEPROM is written to many times
#undef _EEPROMEX_DEBUG

/* EEPROM storage. Each motor gets:
    16-bit int  for speed

    There are 4 motors, so motor data
    is contained in bits 0 to 64

    The entire program gets:
    32-bit int (long) for auto-reverse time in ms
    1 byte for autoReverseOn flag

    Other data starts immediately after the motor data
    on bit number 64, and goes until bit 104
*/

// set data addresses for EEPROM
#define MOTOR_SPEED_DATA_START 0
#define OTHER_DATA_START 64
#define EMPTY_DATA_START 104

// set a default value for auto-reverse
#define AUTO_REVERSE_INTERVAL_DEFAULT 300000L//300000 ms = 5min

//declare objects and variables
AF_DCMotor motors[4] = {AF_DCMotor(1), AF_DCMotor(2), AF_DCMotor(3), AF_DCMotor(4)};
int speeds[4] = {0, 0, 0, 0};
bool motorsOn = true;
bool autoReverseOn = false;
bool motorsReversed = false;
bool readingNumberString = false;
int stringGoesToObject = -1; //an integer flag used for routing multi-character inputs
unsigned long last_reverse_time = 0;
unsigned long wait_for_characters_start = 0;
int seconds_waited = 0;
long autoReverseInterval = AUTO_REVERSE_INTERVAL_DEFAULT;


  // setup runs once when the processor is reset
void setup() {
  // start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeUno
  // Writes before membase or beyond EEPROMSizeUno will only give errors when _EEPROMEX_DEBUG is set
  EEPROM.setMemPool(0, EEPROMSizeUno);

  // Set maximum allowed writes to maxAllowedWrites.
  // Writing > MaxAllowed will only give errors when _EEPROMEX_DEBUG is defined
  EEPROM.setMaxAllowedWrites(10000);

  // start serial communication at 9600 bps
  Serial.begin(9600); 

  // display helpful info for the user
  printHelpInfo();

  //init with current timer value
  last_reverse_time = millis();

  //restore saved user settings
  loadPreviousSettings();

  //display current motor settings
  printStats();
}

/*!
 * 
 */
void loop() {
  //checks for data on the serial port
  checkForUpdates();

  //iterate thorough motors checking speeds
  for (int i = 0; i < N_MOTORS; ++i) {
    checkRunMotor(motors[i], speeds[i]);
  }

  // handle auto-reverse
  unsigned long current_time = millis();
  if ((current_time - last_reverse_time) > autoReverseInterval) {
    if (autoReverseOn) {
      motorsReversed = ! motorsReversed;
    }
    last_reverse_time = current_time;
  }
}

/*!
   Note: motNum is 0-indexed
   Note: motSpeed is a signed 16-bit int that
   contains 8-bit speed data with sign information 
   (-255 to 255).

   Values input to this function are stored in EEPROM
   and restored on power-cycle
*/
void setMotorSpeed(int motNum, int motSpeed) {
  if (motSpeed < 0) {
    motSpeed = -motSpeed;
  }
  if (motSpeed > 255) {
    motSpeed = 255;
    speeds[motNum] = motSpeed;
  }
  //write the speed to the motor driver
  motors[motNum].setSpeed(motSpeed);
  
  //save motor speed
  EEPROM.writeInt(MOTOR_SPEED_DATA_START + (motNum * 16), speeds[motNum]);

  //print new speed to serial
  printMotorSpeed(motNum);
}

/*!
   Note: motSpeed is a signed 16-bit int that
   contains 8-bit speed data and the sign for direction
   range: (-255 to 255)
*/
void checkRunMotor(AF_DCMotor &m, int motSpeed) {
  if (motorsOn) {
    if (motSpeed > 0) {
      motorsReversed ? m.run(BACKWARD) : m.run(FORWARD);
    }
    else if (motSpeed < 0) {
      motorsReversed ? m.run(FORWARD) : m.run(BACKWARD);
    }
    else {
      m.run(RELEASE);
    }
  }
  else {
    m.run(RELEASE);
  }
}

/*!
 * 
 */
void checkForUpdates() {
  //handle new data from serial port
  if (Serial.available()) {
    //handle ASCII ecoded multi-digit decimal number imput
    if (readingNumberString) {
      delay(500);//wait for all serial data to arrive
      processMultiCharacterCommand();
    }
    else {
      processSingleCharacterCommand();
    }
  }
  //print ... if waiting for a multi-character command
  else if (readingNumberString) {
    //print every second
    if ((millis() - wait_for_characters_start) > 1000) {
      Serial.print(".");
      wait_for_characters_start = millis();
      ++seconds_waited;
      if (seconds_waited > 10) {
        seconds_waited = 0;
        readingNumberString = false;
        Serial.println();
        Serial.println("Command timed out.");
        printHelpInfo();
      }
    }
  }
}

/*!
 * This function handles multi-character input.
 * Uses the global flag stringGoesToObject to direct the
 * data to the correct object
 */
void processMultiCharacterCommand() {
  long value = readLongFromSerial();
  readingNumberString = false;
  if (stringGoesToObject < N_MOTORS) {
    if(value < -255) {value = -255;}
    if(value > 255) {value = 255;}
    speeds[stringGoesToObject] = value;
    setMotorSpeed(stringGoesToObject, value);
    Serial.println();
    Serial.print("Set motor ");
    Serial.print(stringGoesToObject + 1);
    Serial.print(" speed to: ");
    Serial.println(value);
  }
  else if (stringGoesToObject = N_MOTORS) {
    if(value < 1000) {value = 1000;}
    autoReverseInterval = value;
    EEPROM.writeLong(OTHER_DATA_START, autoReverseInterval);
    Serial.println();
    Serial.print("Set auto-reverse interval to: ");
    Serial.print(autoReverseInterval);
    Serial.println(" ms");
  }
  else {
    Serial.println("ERROR: unknown error reading multi-character input");
  }
}

long readLongFromSerial() {
  Serial.println();
  uint8_t buf[32] = {0};
  uint8_t i = 0;
  bool isNegative = false;
  while (Serial.available()) {
    char c = Serial.read();
    // if the first character is a minus sign,
    // set negative flag and read another character
    if(i==0 && (int)c == 45) {
      isNegative = true;
      c = Serial.read();
    }
    buf[i] = atoi(&c);
    ++i;
    if (i > 31) {
      Serial.println(F("warning: input exceeds 32 character limit"));
      while (Serial.available()) {
        Serial.read();
      }
      return -1;
    }
  }
  //now i is the number of digits in the base-10 number
  long value = 0;
  --i;
  for (uint8_t j = 0; j <= i; ++j) {
    value += ( (long)(buf[j]) * (long)powint(10, i - j) );
  }
  return isNegative? -1*value : value;
}

/*!
 * 
 */
long powint(int x, int y)
{
  long val = x;
  for (int z = 0; z <= y; z++)
  {
    if (z == 0)
      val = 1;
    else
      val = val * (long)x;
  }
  return val;
}

/*!
 * Handle single-character input from serial terminal
 */
void processSingleCharacterCommand() {
  int data = Serial.read();
  switch (data) {
    //Handle motor 0 data
    case 'q': {
        setMotorSpeed(0, speeds[0]++);
        break;
      }
    case 'a': {
        setMotorSpeed(0, speeds[0]--);
        break;
      }
    case 'z': {
        readingNumberString = true;
        stringGoesToObject = 0;
        Serial.print("Enter speed for motor 1 (value from -255 to 255)...");
        wait_for_characters_start = millis();
        break;
      }
    //Handle motor 1 data
    case 'w': {
        setMotorSpeed(1, speeds[1]++);
        break;
      }
    case 's': {
        setMotorSpeed(1, speeds[1]--);
        break;
      }
    case 'x': {
        readingNumberString = true;
        stringGoesToObject = 1;
        Serial.print("Enter speed for motor 2 (value from -255 to 255)...");
        wait_for_characters_start = millis();
        break;
      }
    //Handle motor 2 data
    case 'e': {
        setMotorSpeed(2, speeds[2]++);
        break;
      }
    case 'd': {
        setMotorSpeed(2, speeds[2]--);
        break;
      }
    case 'c': {
        readingNumberString = true;
        stringGoesToObject = 2;
        Serial.print("Enter speed for motor 3 (value from -255 to 255)...");
        wait_for_characters_start = millis();
        break;
      }
    //Handle motor 3 data
    case 'r': {
        setMotorSpeed(3, speeds[3]++);
        break;
      }
    case 'f': {
        setMotorSpeed(3, speeds[3]--);
        break;
      }
    case 'v': {
        readingNumberString = true;
        stringGoesToObject = 3;
        Serial.print("Enter speed for motor 4 (value from -255 to 255)...");
        wait_for_characters_start = millis();
        break;
      }
    //Handle other data
    case '1': {
        motorsOn = ! motorsOn;
        motorsOn ? Serial.println("Motors on") :
        Serial.println("Motors off");
        break;
      }
    case '2': {
        autoReverseOn = ! autoReverseOn;
        EEPROM.writeByte(OTHER_DATA_START + 32, autoReverseOn);
        autoReverseOn ? Serial.println("Auto-reverse on") :
        Serial.println("Auto-reverse off");
        break;
      }
    case '3': {
        readingNumberString = true;
        stringGoesToObject = N_MOTORS;
        Serial.print("Enter time between auto-reverse (units = ms)...");
        wait_for_characters_start = millis();
        break;
      }
    case '?': {
        printAllStats();
        break;
      }
    default: {
        printHelpInfo();
      }
  }
}

/*!
   This function is called in setup to restore the
   previous setting after a power-cycle
*/
void loadPreviousSettings() {
  //read motor speeds and directions
  for (int i = 0; i < N_MOTORS; ++i) {
    speeds[i] = EEPROM.readInt(MOTOR_SPEED_DATA_START + (i * 16));
    setMotorSpeed(i, speeds[i]);
  }

  //read autoReverse settings
  autoReverseInterval = EEPROM.readLong(OTHER_DATA_START);
  if(autoReverseInterval < 0) {
    autoReverseInterval = AUTO_REVERSE_INTERVAL_DEFAULT;
  }
  autoReverseOn = EEPROM.readByte(OTHER_DATA_START + 32);
}

/*************************************************************
 *************************************************************
   Functions to print user data
 *************************************************************
*************************************************************/

/*!
 * Print the speed of a motor
 * Note: motor is 0-indexed
 */
void printMotorSpeed(int motor) {
  Serial.print("motor ");
  Serial.print((motor + 1));
  Serial.print(" speed: ");
  Serial.print(speeds[motor]);
  Serial.print("\t(");
  Serial.print((float)(speeds[motor] * 100) / 255.0);
  Serial.println("%)");
}

/*!
 * 
 */
void printAllStats() {
  Serial.println(F("*************************************"));
  Serial.println(F("***********System Stats**************"));
  Serial.println(F("*************************************"));
  printAllMotorSpeeds();
  printStats();
}

void printAllMotorSpeeds() {
  for (int i = 0; i < N_MOTORS; ++i) {
    printMotorSpeed(i);
  }
}

void printStats() {
  Serial.print("Motors on?: ");
  Serial.println(motorsOn ? "Y" : "N");
  Serial.print("Auto-reverse duration (ms): ");
  Serial.println(autoReverseInterval);
  Serial.print("Time until next auto-reverse: (ms): ");
  Serial.println(autoReverseInterval-(millis() - last_reverse_time));
  Serial.print("Motor direction: ");
  Serial.println(motorsReversed ? "Rev" : "Fwd");
  Serial.print("Auto-reverse on?: ");
  Serial.println(autoReverseOn ? "Y" : "N");
}

/*!
 * Prints after processor is reset
 */
void printHelpInfo() {
  Serial.println(F("*************************************"));
  Serial.println(F("*************************************"));
  Serial.println(F("DC Motor Controller V1.6"));
  Serial.println(F("Designed by Huilab in California (C)"));
  Serial.println(F("*************************************"));
  Serial.println(F("Press 'q' or 'a' to accel/decel M1. Press z to set the speed of M1"));
  Serial.println(F("Press 'w' or 's' to accel/decel M2. Press x to set the speed of M2"));
  Serial.println(F("Press 'e' or 'd' to accel/decel M3. Press c to set the speed of M3"));
  Serial.println(F("Press 'r' or 'f' to accel/decel M4  Press v to set the speed of M4"));
  Serial.println(F("Press 1 to toggle all motors on/off."));
  Serial.println(F("Press 2 to toggle auto reverse on/off."));
  Serial.println(F("Press 3 to set auto-reverse interval (ms)"));
  Serial.println(F("Press ? to get motor information"));
  
  Serial.println(F("*************************************"));
  Serial.print(F("**NOTE: motor speeds, auto-reverse on/off, and \nauto-reverse duration"));
  Serial.println(F(" are saved and \nmotors will resume at the saved speed on next use"));
  Serial.println(F("*************************************"));

  Serial.print(F("**NOTE: Do not unplug motors while the board is powered on.\n"));
  Serial.println(F("Doing so may damage the microprocessor"));
  Serial.println(F("*************************************"));
}

