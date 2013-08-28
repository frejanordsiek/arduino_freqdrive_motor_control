/* Copyright (c) 2013, Freja Nordsiek
   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:
   
     Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
   
     Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.
   
     Neither the name of the {organization} nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Frequency Drive Motor Control
   
   Controls up to four frequency drive driven motors where the
   frequency drives are controlled by two digital signals (a
   Start/Stop signal and a Forward/Reverse signal) and an
   analog voltage signal to control the frequency where the
   analog signals are made with an Analog Devices AD56X4 series
   Quad DAC (communicating over SPI. Motor settings are set by
   sending messages over the serial connection to the Arduino.
   If the arduino has to wait longer than a specified timeout, it
   stops all motors.
   
   Author:   Freja Nordsiek
   Notes:
   History:  * 2013-08-16 Created.
*/

/* There are four different commands that are accepted over the
   serial link. An invalid command causes a response of "Invalid".
   
   Check Program Status:
     "Status?"
     
     Request the status of this motor control program. Responds "OK"
     if the program is working right.
   
   Return Motor Settings:
     "MotorSettings?"
     
     Request the Start/Stop, Forward/Reverse, and frequency set
     voltages of each motor. A string of the form "XXX YYY A B C" is
     the response where the X's are the Start/Stop states of the
     motors ('A' for start and the letter 'O' for stop), the Y's are
     the Forward/Reverse states of the motors ('F' for forward and
     'R' for reverse), and A B and C are floating point numbers giving
     the frequency set voltages of the motors.
   
   Halt Motors:
     "Halt"
     
     Stops all motors. Responds with "ACK".
   
   Set Motor Settings:
     "SetMotors: XXX YYY A B C"
     
     Set the motor settings. String works the same way as the returned
     motor setting status for the Return Motor Settings command. If
     the command is valid, the motor settings are updated and a
     response of "ACK" is sent. If the command is invalid, the current
     motor settins are kept and a response of "Invalid" is sent. 
*/

#include "Arduino.h"
#include "math.h"
#include "ctype.h"
#include <SPI.h>
#include <AD56X4.h>

// Output pin for the Slave Select SPI line of the AD56X4.

int AD56X4_SS_pin = 10;

/* A string is needed to hold serial commands sent by the computer
   along with a flag to indicate whether the command is complete
   or not (not complete till it is terminated with a newline).
   This scheme is done so that the program is still functioning
   while waiting for commands from the computer. Also, the max
   length of the command string needs to be known.
*/

String commandFromComputerString = "";
boolean commandFromComputerComplete = false;
int commandFromComputerMaxLength = 256;

/* It is important that the motors be stopped if we don't receive
   any commands from the computer in a while. So, we need to define
   a timeout in milliseconds and have a variable to store the last
   time a command was received.
*/

unsigned long timeout = 10000;
unsigned long timeOfLastCommand;

// How many motors we have.

int numberMotors = 2;

// Arrays to hold the output pins for the Start/Stop and
// Forward/Reverse controls of all 4 motors (will only use the
// first numberMotors).

int motorStartStopPins[] = {3, 4, 6, 7};
int motorForwardReversePins[] = {2, 5, 8, 9};

// Arrays to hold the max voltages to be applied to the motor
// frequency controls, the slopes of the voltage to DAC value
// lines, and the y-intercept of the voltage to DAC value lines.

float motorFrequencyMaxVoltages[] = {7, 11, 10, 10};
float motorFrequencyVoltageToDACslope[] = {0.0001674281,0.0001677479,0.00015259,0.00015259};
float motorFrequencyVoltageToDACintercept[] = {0.005382222, 0.01291596, 0, 0};

// Arrays to hold the Start/Stop (true is start), Forward/Reverse
// (true is reverse), and frequency control set voltage states
// of the motors (true is start and reverse). The initial state
// will be stop, reverse, set voltage of zero.

boolean motorStartStates[] = {false, false, false, false};
boolean motorReverseStates[] = {false, false, false, false};
float motorFrequencySetVoltages[] = {0, 0, 0, 0};

// Need an array to hold the DAC values to be output by the DAC.
word motorFrequencyDACvalues[] = {0, 0, 0, 0};

void setup()
{
  
  // Setup SPI. This means setting pin 10 to output (arduino must
  // be the master), the AD56X4 Slave Select pin to output, the
  // SPI clock (chip's 50 MHz is way faster than our 8 MHz max),
  // and start SPI.
  
  pinMode(10,OUTPUT);
  pinMode(AD56X4_SS_pin,OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();
  
  // Reset the AD56X4, which will power it up and set all outputs
  // to zero.
  
  AD56X4.reset(AD56X4_SS_pin,true);
  
  // Set the start/stop and forward/reverse pins to output.
  
  for (int i = 0; i < numberMotors; i++)
    {
      pinMode(motorStartStopPins[i], OUTPUT);
      pinMode(motorForwardReversePins[i], OUTPUT);
    }
  
  // Set the motor states to their initial values.
  
  WriteMotorControlStates();
  
  // Reserve sufficient space in the motor command string for
  // commands.
  
  commandFromComputerString.reserve(commandFromComputerMaxLength);
  
  // The present time will be defined as the time of the first
  // command as far as timeout purposes.
  
  timeOfLastCommand = millis();
  
  // We want to communicate with the computer at the fastest speed
  // possible.
  
  Serial.begin(115200);
  
}

void loop()
{
  
  // If we have a complete command from the serial, it needs to be
  // processed.
  
  if (commandFromComputerComplete)
    {
      
      // As we have a new command, we need to set timeOfLastCommand
      // to now.
      
      timeOfLastCommand = millis();
    
      // Trim off the newline and any characters after it (will
      // only deal with one command at a time).
      
      commandFromComputerString = commandFromComputerString.substring(0,commandFromComputerString.indexOf('\n'));
      
      // Check the command string for each of the four commands in
      // turn, do the appropriate command, or respond with "Invalid"
      // if it was not a valid command.
      
      if (commandFromComputerString == "Status?")
        {
          // Only status we have so far is just OK.
          Serial.print("OK\n");
        }
      else if (commandFromComputerString == "MotorSettings?")
        {
          
          // To save memory, the return string will just be put
          // into the command string. So, it is cleared and then
          // the Start/Stop states are put in followed by the
          // Forward/Reverse states and finally the frequency set
          // voltages (have spaces between them unlike all the
          // Start/Stops and Forward/Reverses).
          
          commandFromComputerString = "";
          
          for (int i = 0; i < numberMotors; i++)
            {
              if (motorStartStates[i])
                commandFromComputerString += "A";
              else
                commandFromComputerString += "O";
            }
          
          commandFromComputerString += " ";
          
          for (int i = 0; i < numberMotors; i++)
            {
              if (motorReverseStates[i])
                commandFromComputerString += "R";
              else
                commandFromComputerString += "F";
            }
          
          commandFromComputerString += " ";
              
          for (int i = 0; i < numberMotors; i++)
            {
              commandFromComputerString += floatToString(motorFrequencySetVoltages[i]);
              if (i < numberMotors - 1)
                commandFromComputerString += " ";
            }
          
          commandFromComputerString += "\n";
          
          Serial.print(commandFromComputerString);
          
        }
      else if (commandFromComputerString == "Halt")
        {
          // Stop all motors.
          for (int i = 0; i < numberMotors; i++)
            {
              motorStartStates[i] = false;
              motorReverseStates[i] = false;
              motorFrequencySetVoltages[i] = 0;
            }
          Serial.print("ACK\n");
        }
      else if (commandFromComputerString.startsWith("SetMotors: "))
        {
          if (processMotorSetCommand(commandFromComputerString))
            Serial.print("ACK\n");
          else
            Serial.print("Invalid\n");
        }
      else // As it wasn't a recognized command, return "Invalid".
        Serial.print("Invalid\n");
    
    // Clear the command string and reset the flag saying that
    // the command is complete (that a new command is waiting to
    // be processed).
    
    commandFromComputerString = "";
    commandFromComputerComplete = false;
    
  }
  
  // If the time since the last command is greater than the
  // timeout, we need to stop all motors.
  
  if (millis() - timeOfLastCommand > timeout)
    {
      for (int i = 0; i < numberMotors; i++)
        {
          motorStartStates[i] = false;
          motorReverseStates[i] = false;
          motorFrequencySetVoltages[i] = 0;
        }
    }
  
  // Send the Start/Stop, Forward/Reverse, and frequency set
  // voltage states to the motor drives.
  
  WriteMotorControlStates();
  
}

/* Given the set command string for the motors, the command is
   validated and the start/stop, forward/reverse, and frequency set
   votage arrays set to the new values in the command if the command
   is valid (discarded otherwise). Returns whether the command was
   valid (true) or not (false).
   
   A valid motor command looks like
   
   "SetMotors: XXX YYY 2.13 3.19e1 -1e-1"
   
   It must start with "SetMotors: " followed by the start/stop
   states for each motor ('A' for start and the letter 'O' for stop).
   Then, a space followed by the forward/reverse states of each
   motor ('F' for forward and 'R' for reverse). Then the frequency
   set voltages for each motor as floats separated by spaces with no
   trailing space.
 */
 boolean processMotorSetCommand(String s)
{
  
  // The command must start right.
  
  if (!commandFromComputerString.startsWith("SetMotors: "))
    return false;
  
  // Arrays are needed to hold the new motor states as we read them
  // before copying them over only if the whole command is valid.
  
  boolean newStartStates[4];
  boolean newReverseStates[4];
  float newFrequencySetVoltages[4];
  
  // Need a temporary character variable when reading the start/stop
  // and forward/reverse states.
  
  char temp;
  
  // Strip the leading command from the front of the string. The
  // command is "SetMotors: " which is 11 characters long.
  
  s = s.substring(11);
  
  // We need numberMotor+1+numberMotor characters to have all the
  // start/stop and forward/reverse states with a space in between.
  // Otherwise, the command was invalid.
  
  if (s.length() < 1 + 2*numberMotors)
    return false;
    
  // Read the start/stop states and check for validity.
  
  for (int i = 0; i < numberMotors; i++)
    {
      temp = s.charAt(i);
      if (temp != 'A' && temp != 'O')
        return false;
      newStartStates[i] = (temp == 'A');
    }
    
  // Check to see if there is an intervening space.
  
  if (s.charAt(numberMotors) != ' ')
    return false;
  
  // Read the forward/reverse states and check for validity.
  
  for (int i = 0; i < numberMotors; i++) 
    {
      temp = s.charAt(numberMotors+1+i);
      if (temp != 'F' && temp != 'R')
        return false;
      newReverseStates[i] = (temp == 'R');
    }
    
  // Check to see if there is an intervening space before the
  // frequency set voltages.
  
  if (s.charAt(2*numberMotors + 1) != ' ')
    return false;
    
  // Remove the start/stop and forward/reverse parts of the command.
  
  s = s.substring(2*numberMotors + 2);
    
  // Read the frequency set voltages and check for validity. After
  // each voltage is read, that part of the string is stripped out
  // from s.
  
  for (int i = 0; i < numberMotors; i++)
    {
      float voltage;
      int index;
      
      // If we are on the last motor, we can use the whole string. If
      // not, we must find the first space (if there isn't the command
      // is invalid) and use the substring up to that space.
      
      if (i == numberMotors - 1)
        voltage = stringToFloat(s);
      else
        {
          index = s.indexOf(' ');
          if (index == -1)
            return false;
          voltage = stringToFloat(s.substring(0,index));
        }
      
      // If the voltage is infinite or NaN (could mean something went
      // wrong with reading it), the command is not valid.
      
      if (isnan(voltage) || isinf(voltage))
        return false;
      
      // Store the voltage.
        
      newFrequencySetVoltages[i] = voltage;
      
      // If there is another motor, the string we just used to get
      // the voltage needs to be stripped out. We also need to check
      // that there are any extra characters after the first space.
      // Otherwise, there is no more voltage strings.
      
      if (i != numberMotors - 1)
        {
          if (s.length() < index + 2)
            return false;
           s = s.substring(index+1);
        }
    }
    
  // Everything was valid, so now we need to copy it over and return
  // that everything was valid.
  
  for (int i = 0; i < numberMotors; i++)
    {
      motorStartStates[i] = newStartStates[i];
      motorReverseStates[i] = newReverseStates[i];
      motorFrequencySetVoltages[i] = newFrequencySetVoltages[i];
    }
  
  return true;
  
}

// Set the Start/Stop, Forward/Reverse, and frequency control set
// voltage states of the motor drives correctly.
void WriteMotorControlStates()
{
  
  // Iterate over each motor.
  
  for (int i = 0; i < numberMotors; i++)
    {
      
      // If the motor is supposed to be start, the output should
      // be HIGH. Stop is LOW.
      
      if (motorStartStates[i])
        digitalWrite(motorStartStopPins[i], HIGH);
      else
        digitalWrite(motorStartStopPins[i], LOW);
      
      // If the motor is supposed to be going forward, the output
      // should be LOW. Reverse is HIGH.
      
      if (motorReverseStates[i])
        digitalWrite(motorForwardReversePins[i], LOW);
      else
        digitalWrite(motorForwardReversePins[i], HIGH);
        
      // For this motor, we need to output the proper voltage by
      // forcing it into the range [0, motorFrequencyMaxVoltage].
      
      float voltage = constrain(motorFrequencySetVoltages[i],0,
                                motorFrequencyMaxVoltages[i]);
        
      // The slope and intercept of the voltage to DAC output curve
      // is used to calculate the DAC output, which is stored in
      // motorFrequencyDACvalues for later writing to the DAC. To
      // insure that there is no wrapping, it is calculated first as
      // a long (32-bit signed integer) before being forced into the
      // range [0, 0xFFFF] to put in the array.
        
      long outputValue = lround((voltage
                          - motorFrequencyVoltageToDACintercept[i])
                          / motorFrequencyVoltageToDACslope[i]);
      
      motorFrequencyDACvalues[3-i] = word(constrain(outputValue,0,0xFFFF));
      
    }
  
  // Write the DAC outputs making sure that both the input (buffer)
  // and DAC (output) registers are updated.
  
  AD56X4.setChannel(AD56X4_SS_pin,AD56X4_SETMODE_INPUT_DAC,
                    motorFrequencyDACvalues);
  
}

/* This is basically copied from the SerialEvent example with just
   formatting and naming changes.
   
   This is called after each iteration of loop if there is any new
   data on the serial RX line. All available bytes are grabbed and
   appended onto commandFromComputerString, which is marked as
   complete if a newline character was received.
*/
void serialEvent()
{
  
  // Grab each and every byte available.
  
  while (Serial.available())
    {
      
      // Read the first available char.
      
      char currentChar = (char)Serial.read();
     
     // Append the char to the command received so far if there is
     // room and if there is no room, move all the characters down
     // one slot and then add it.
      
      if (commandFromComputerString.length() < commandFromComputerMaxLength)
        commandFromComputerString += currentChar;
      else
        {
          for (int i = 0; i < commandFromComputerString.length() - 1; i++)
            {
              commandFromComputerString.setCharAt(i,commandFromComputerString.charAt(i+1));
            }
          
          commandFromComputerString.setCharAt(commandFromComputerString.length() - 1,currentChar);
        }
      
      // If the character is a newline, mark the command as complete.
      
      if (currentChar == '\n')
        commandFromComputerComplete = true;
      
    }
  
}


// Converts a float to a string.
String floatToString(float x)
{
  // We have to use the avr-libc function dtostr, which requires
  // a character array buffer. We are using the flag 0x02 which causes
  // the sign (+ or -) to always be printed.
  
  char buf[256];
  dtostre(x,buf,8,0x02);
  return String(buf);
  
}

// Converts a string to a float.
float stringToFloat(String s)
{
  
  /* Rather than using the builtin strtod, I am implementing my own
     to handle more ways to give special values and to also allow
     'd' and 'D' to be used in place of 'e' or 'E'.
  */
  
  // All the surrounding whitespace needs to be removed, and
  // converting to lower case will make for less character testing.
  // Then convert all 'd' characters to 'e'.
  
  s.trim();
  s.toLowerCase();
  s.replace('d','e');
  
  // First, we need to look for the special cases of NaN and different
  // ways of writing infinity. Then, we have to handle the more
  // general case.
  
  if (s.length() == 0 || s == "nan" || s == "notanumber")
    return NAN;
  else if (s == "inf" || s == "infty" || s == "infinity")
    return INFINITY;
  else if (s == "-inf" || s == "-infty" || s == "-infinity")
    return -INFINITY;
  else
    {
      
      // Need to know if the number is negative or positive as well
      // as finding the indices of the decimal point and exponent
      // character ('e') if they are there.
      
      boolean isNegative = s.startsWith("-");
      int decimalIndex = s.indexOf(".");
      int exponentIndex = s.indexOf("e");
      
      // If there are any other decimal points or exponent characters,
      // or the exponent isn't after the decimal, the string is
     //  invalid and we should return NaN.
      
      if ((decimalIndex != -1 && decimalIndex != s.lastIndexOf('.'))
          || (exponentIndex != -1 && exponentIndex != s.lastIndexOf('e'))
          || (decimalIndex != -1 && exponentIndex != -1
              && decimalIndex > exponentIndex))
        return NAN;
        
      // The value will be steadily constructed from zero.
      
      float value = 0;
      
      /* First, we need to get the part before the decimal point. The
         characters from the beginning to the end of the part in front
         of the decimal need to be checked and read one by one. So,
         bounds on the indices are needed. The starting one (inclusive)
         is 0 unless there is a sign in front in which case it is 1.
         The end index (exclusive) is just the location of the decimal
         point if present, the location of the exponent if not, and
         finally the length of the string if that isn't present
         either.
      */
      
      int indexStart = (isNegative || s.startsWith("+")) ? 1 : 0;
      
      int indexEnd = s.length();
      
      if (decimalIndex != -1)
        indexEnd = decimalIndex;
      else if (exponentIndex != -1)
        indexEnd = exponentIndex;
      
      // Read the digits one by one before the decimal/exponent/end.
      
      for (int i = indexStart; i < indexEnd; i++)
        {
          
          /* If the i'th character is not a digit, then the string was
             invalid and we should return NaN. If it is, then we
             should multiply the current value by 10 (move it over a
             place) and add the digit (it is an ASCII character and
             the digits are in 0-9 order with '0' having the value
             48).
          */
          
          char c = s.charAt(i);
          
          if (!isdigit(c))
            return NAN;
            
          value = 10*value + (c - 48);
        }
        
      // If we have a decimal point, we need to read the decimal part.
        
      if (decimalIndex != -1)
        {
          
          // The value of the decimal part will be constructed
          // separately before adding it to value.
          
          float decimalPart = 0;
          
          /* To read the characters from the decimal point to the
             exponent or end, the start index (inclusive) and end
             index (exclusive) are needed. The start index is just
             right after the decimal point. The end index is either
             the exponent index, or if it is not there, the string
             length.
           */
          
          indexStart = decimalIndex + 1;
          indexEnd = (exponentIndex != -1) ? exponentIndex : s.length();
          
          // Read the digits of the decimal part one by one.
          
          for (int i = indexStart; i < indexEnd; i++)
            {
              
              /* If the i'th character is not a digit, then the string
                 was invalid and we should return NaN. If it is, then
                 we should multiply the current decimal part by 10
                 (move it over a place) and add the digit (it is an
                 ASCII character and the digits are in 0-9 order with
                 '0' having the value 48).
            */
              
              char c = s.charAt(i);
              if (!isdigit(c))
                return NAN;
              decimalPart = 10*decimalPart + (c - 48);
            }
          
          // Now the decimal part needs to be added to the current
          // running value. We must first divide by enough powers of
          // 10 that the decimal part is truly behind the decimal
          // point (one for each 10 we multiplied and then one more).
            
          value += decimalPart * pow(10,-(indexEnd-indexStart));

        }
      
      // If we have an exponent, construct it and apply it.
      
      if (exponentIndex != -1)
        {
          
          // The value of the exponent part will be constructed
          // separately before multiplying value by the right power
          // of 10.
          
          float exponentPart = 0;
          boolean exponentNegative = false;
          
          // The starting index is right after the exponent, unless
          // there is a + or - sign in which case it is the next one.
          // Also, if there is a negative sign, exponentNegative
          // needs to be set.
          
          indexStart = exponentIndex + 1;
          
          if (indexStart < s.length() && s.charAt(indexStart) == '+')
            indexStart++;
          else if (indexStart < s.length() &&  s.charAt(indexStart) == '-')
            {
              indexStart++;
              exponentNegative = true;
            }
          
          // Read the digits from the exponent all the way to the end.
          
          for (int i = indexStart; i < s.length(); i++)
            {
              
              /* If the i'th character is not a digit, then the string
                 was invalid and we should return NaN. If it is, then
                 we should multiply the current decimal part by 10
                 (move it over a place) and add the digit (it is an
                 ASCII character and the digits are in 0-9 order with
                 '0' having the value 48).
            */
              
              char c = s.charAt(i);
              if (!isdigit(c))
                return NAN;
              exponentPart = 10*exponentPart + (c - 48);
            }
          
          // Now the final float represented by the string is obtained
          // by multiplying by 10 to the power of exponentPart (after
          // setting the sign of course).
          
          if (exponentNegative)
            exponentPart *= -1;
            
          value *= pow(10,exponentPart);
          
        }
      
      // All that is left is to set the sign.
      
      return isNegative ? -value : value;
      
    }
    
}


