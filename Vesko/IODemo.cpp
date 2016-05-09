#include "stdafx.h"

#include "ldcncom.h"
#include "io.h"
#include "IODemo.h"

#include <iostream>
#include <fstream>
using namespace std;


void ReadModuleData(byte addr)
{
byte indx;
byte tmrmode;
unsigned long timeval;

// Read selected status items
// and store received data into internal data structure

  LdcnReadStatus(addr, SEND_INPUTS | SEND_AD1 | SEND_AD2 | SEND_AD3 | SEND_TIMER);
  
// Get module status data

  for( indx = 0; indx < 16; indx++) {
// Update input bit values
    cout << "Input " << indx << " = " << IoInBitVal(addr, indx) << "\n";
// Update output bit values
    cout << "Output " << indx << " = " << IoOutBitVal(addr, indx) << "\n";
  }
  
// Update A/D values
  for(indx = 0; indx < 3; indx++) 
    cout << "AD " << indx << " = " <<  IoGetADCVal(addr, indx)  << "\n";
  

// Update timer value
  tmrmode = IoGetTimerMode(addr);
  timeval = IoGetTimerVal(addr);
  
  switch (tmrmode) {
    case TIMERMODE | RESx1:
      cout << timeval * 0.0000002 << " sec\n";
      break;
    case TIMERMODE | RESx2:
      cout << timeval * 0.0000004 << " sec\n";
      break;
    case TIMERMODE | RESx4:
      cout << timeval * 0.0000008 << " sec\n";
      break;
    case TIMERMODE | RESx8:
      cout << timeval * 0.0000016 << " sec\n";
      break;
    case COUNTERMODE | RESx1:
    case COUNTERMODE | RESx2:
    case COUNTERMODE | RESx4: 
    case COUNTERMODE | RESx8:
      cout << timeval << " counts\n";
  }
}

// Set / Clear Outport bit
void SetOutput(byte addr, int index )
{
  if (index > 7) return;
  
  if (!IoOutBitVal(addr, index))
     IoSetOutBit(addr, index);
  else
     IoClrOutBit(addr, index);
}


void SetPWM(byte addr, byte pwm1, byte pwm2) 
{
// Immediately set the two PWM output values.
// To use OUTPUT 1, 2 in PWM mode set output byte
// bits 1 and 2 (Set Output command) to 1.
// A value of 255 will turn off the PWM output;
// a value of 0 will turn it on with a 100% duty cycle.

  IoSetPWMVal(addr, pwm1, pwm2);
  IoSetOutBit(addr, 1);
  IoSetOutBit(addr, 2);
  
}


bool SetTimerMode(byte addr, byte mode, byte resolution)
{
byte tmrmode;
  
  if ((mode == COUNTERMODE) || (mode == TIMERMODE) || (mode == OFFMODE))
      tmrmode = mode;
  else {   
      cout << "Invalid Timer Mode parameter\n";
      return false;
  }

  if ((resolution == RESx1)||(resolution == RESx2)||(resolution == RESx4)||(resolution == RESx8))
     tmrmode = tmrmode | resolution;
  else {
     cout << "Invalid Timer Mode parameter\n";
     return false;
  }
  
  return bool(IoSetTimerMode(addr, tmrmode));
}


void IODemo(char addr)
{
bool result;

     
  result = SetTimerMode(addr, TIMERMODE, RESx2);

  if (!result) return;
       
  SetPWM(addr, 0, 200);
  
  SetOutput(addr, 3);
  SetOutput(addr, 4);
  SetOutput(addr, 14);
  
  ReadModuleData(addr);
  
}
