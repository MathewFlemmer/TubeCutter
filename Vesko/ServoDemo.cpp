#include "stdafx.h"

#include "ldcncom.h"
#include "servo.h"

#include <iostream>
#include <fstream>
using namespace std;



extern byte Module_ID[MAXNUMMOD + 1];

/////////////////////////////////////////////////////////////////////////////

bool IsServoOn(byte addr)
{
byte status, auxb, stopctrl;

// Read Status and Auxiliary bytes from module
   LdcnReadStatus(addr, SEND_AUX);

// Get data from the internal data structure
   status = LdcnGetStat(addr);
   auxb = ServoGetAux(addr);
   stopctrl = ServoGetStopCtrl(addr);

   return (auxb & SERVO_ON) && (status & POWER_ON) && (stopctrl & SRV_ENABLE_AMP);
}
/////////////////////////////////////////////////////////////////////////////

bool TurnServoOn(byte addr)
{
byte status;

   ServoStopMotor(addr, MOTOR_OFF);     // Stop accumulating position error
   ServoStopMotor(addr, STOP_SMOOTH);   // Clear position error

// Read Status byte from module
   LdcnReadStatus(addr, SEND_AUX);

// Get data from the internal data structure
   status = LdcnGetStat(addr);
  
// Check up diagnostic bits
  if ((status & LIMIT1) && (status & LIMIT2) && (status & POWER_ON)) 
// And turn servo on only if the motor is in good condition
     return ServoStopMotor(addr, STOP_SMOOTH | SRV_ENABLE_AMP) &&
            ServoClearBits(addr);
  else return 0;
}
/////////////////////////////////////////////////////////////////////////////

void TurnServoOff(byte addr)
{
  if (IsServoOn(addr))                 // be sure the motor is not in motion
	 ServoStopMotor(addr, SRV_ENABLE_AMP | STOP_ABRUPT);

  ServoStopMotor(addr, MOTOR_OFF);
  ServoClearBits(addr);
}
/////////////////////////////////////////////////////////////////////////////

void Servo_StopMotor(byte addr, byte stop_mode)
{
byte mode;
  
  mode = ServoGetStopCtrl(addr);   
  switch (stop_mode) {
// no change of the amplifier status
    case STOP_SMOOTH: mode = mode & SRV_ENABLE_AMP | STOP_SMOOTH; break;
    case STOP_ABRUPT: mode = mode & SRV_ENABLE_AMP | STOP_ABRUPT; break;
    case MOTOR_OFF  : mode = mode & SRV_ENABLE_AMP | MOTOR_OFF;   break;
    default: return;
  }
  ServoStopMotor(addr, mode);   //Send stop command
}
/////////////////////////////////////////////////////////////////////////////

bool StartHoming(byte addr, byte mode)
{
byte status;

  LdcnNoOp(addr);
  status = LdcnGetStat(addr);

  if ((mode & ON_CUR_ERR) && (status & OVERCURRENT))  {
     cout << "Overcurrent flag must be cleared before homing on overcurrent\n";
     return false;
  }
  
  if ((mode & ON_POS_ERR) && (status & POS_ERR))      {
     cout << "Position error flag must be cleared before homing on position error\n";
     return false;
  }
    
  if ( mode == 0 ) {
     cout << "No homing criterion selected\n";
     return false;
  }

  return ServoSetHoming(addr, mode);
}
/////////////////////////////////////////////////////////////////////////////

void FindIndex(byte addr)
{
byte mode;

// Start any motion with parameters:
//   position = 0 - not used because bit LOAD_POS ( &H1 ) of the mode parameter is 0
//   velocity = 80000
//   acceleration = 20000
//   PWM = 0 - not used because bit LOAD_PWM ( &H8 ) of the mode parameter is 0
//   mode is combination of LOAD_ACC, LOAD_VEL, ENABLE_SERVO, VEL_MODE and START_NOW bits
  
    mode = LOAD_ACC | LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

    ServoLoadTraj(addr, mode, 0, 80000, 20000, 0);
  
    
//  Start homing - the motor will stop abrupt when index is found
    
    mode = ON_INDEX | HOME_STOP_ABRUPT;
    StartHoming(addr, mode);
}
/////////////////////////////////////////////////////////////////////////////

void StartPositionMotion(byte addr, long position, long velocity, long acceleration, bool start_now)
{
byte mode;

  mode = LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO;
  if (start_now) mode |= START_NOW;
  if (velocity < 0) velocity = -velocity;
  ServoLoadTraj(addr, mode, position, velocity, acceleration, 0);
}
/////////////////////////////////////////////////////////////////////////////

void StartVelocityMotion(byte addr, long velocity, long acceleration, bool start_now)
{
byte mode;

  mode = LOAD_VEL | LOAD_ACC | VEL_MODE | ENABLE_SERVO;
  if (start_now) mode |= START_NOW;
  if (velocity < 0) {
      velocity = -velocity;
      mode |= REVERSE;
  }
  ServoLoadTraj(addr, mode, 0, velocity, acceleration, 0);
}
/////////////////////////////////////////////////////////////////////////////

void StartPWMMotion(byte addr, short int pwm, bool start_now)
{
byte mode;

  mode = LOAD_PWM;
  if (start_now) mode |= START_NOW;
  if (pwm < 0) {
      pwm = -pwm;
      mode |= REVERSE;
  }
  ServoLoadTraj(addr, mode, 0, 0, 0, byte(pwm));
}
/////////////////////////////////////////////////////////////////////////////

void GoToPosition(byte addr, long position, long velocity, long acceleration)
{
byte status;

  StartPositionMotion(addr, position, velocity, acceleration, true);
  do {
    LdcnNoOp(addr);
    status = LdcnGetStat(addr);
  } while (!(status & MOVE_DONE) && IsServoOn(addr));
}
/////////////////////////////////////////////////////////////////////////////

void GetDeviceID(byte nummod) 
{
byte type, version, inport_3;

  for (int addr = 1; addr <= nummod; addr++) {  // for all modules found

     type = LdcnGetModType(addr);
     version = LdcnGetModVer(addr);

     if (type == SERVOMODTYPE) { //servo

       ServoStopMotor(addr, 0x40);  // if StopMode bit 6 == 1 Inport3 == Device_ID

       LdcnReadStatus(addr, SEND_INPORTS);

       inport_3 = ServoGetInport3(addr);

       Module_ID[addr] = (inport_3 < 32 ? 0 : inport_3);

       ServoStopMotor(addr, 0x00);

       if (Module_ID[addr] == 0)
         if (version >= 60 && version < 70) Module_ID[addr] = 182; else
         if (version < 60) Module_ID[addr] = 173; else
         if (version >= 70 && version < 80) Module_ID[addr] = 174;
     }
     else  Module_ID[addr] = 0; // Unknown device
   }
}
/////////////////////////////////////////////////////////////////////////////

int Diagnose(byte addr)
{
byte stat, auxb, stopctrl;
int DiagResult = 0;

// Read Status and Auxiliary bytes from module
   LdcnReadStatus(addr, SEND_AUX);

// Get data from the internal data structure
   stat = LdcnGetStat(addr);
   auxb = ServoGetAux(addr);
   stopctrl = ServoGetStopCtrl(addr);


  if (stopctrl & SRV_ENABLE_AMP) {
     if (!(stat & LIMIT2) &&  (stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |=  64;
     if ( (stat & LIMIT2) && !(stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |= 128;
     if (!(stat & LIMIT2) && !(stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |= 192;
     if (!(stat & LIMIT2) &&  (stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |=  16;
     if (!(stat & LIMIT2) && !(stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |=  32;
     if ( (stat & LIMIT2) &&  (stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |=   4;
     if ( (stat & LIMIT2) && !(stat & LIMIT1) && !(stat & POWER_ON))
        if (stat == 0x51)
           if   (auxb & INDEX) DiagResult |= 2;
           else                DiagResult |= 256;
        else DiagResult |= 8;
  } else {
     if ( (stat & LIMIT2) &&  (stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |= 1;
     if ( (stat & LIMIT2) && !(stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |= 2;
     if (!(stat & LIMIT2) &&  (stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |= 4;
     if ( (stat & LIMIT2) && !(stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |= 3;
     if (!(stat & LIMIT2) &&  (stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |= 5;
     if (!(stat & LIMIT2) && !(stat & LIMIT1) &&  (stat & POWER_ON)) DiagResult |= 6;
     if (!(stat & LIMIT2) && !(stat & LIMIT1) && !(stat & POWER_ON)) DiagResult |= 7;
  }

  if (DiagResult & 16 & 1) DiagResult &= ~(16);
   
  return DiagResult; 
}
/////////////////////////////////////////////////////////////////////////////

void ReportError(byte addr, int DiagResult)
{
  if (DiagResult &   1)                                        //   1
    switch (Module_ID[addr]) {
      case 163: cout << "UNDER/OVERVOLTAGE\n";      break;
      case 160:
      case 180: cout << "SERVO AMPLIFIER FAULT\n";  break;
      default : cout << "OVERVOLTAGE\n";            break;
    }

  if (DiagResult &   2) cout << "STOP IN ACTIVATED\n";         //   2
  if (DiagResult &   4) cout << "OVERHEAT\n";                  //   4
  if (DiagResult &   8) cout << "STOP IN or ENCODER ERROR\n";  //   8

  if (DiagResult &  16)                                        //  16
     switch (Module_ID[addr]) {
       case 163: cout << "MOTOR SHORT or UNDER/OVERVOLTAGE\n"; break;
       case 160:
       case 180: cout << "SERVO AMPLIFIER FAULT\n";            break;
       default : cout << "MOTOR SHORT or OVERVOLTAGE\n";       break;
     }

  if (DiagResult &  32) cout << "OVERCURRENT\n";               //  32
  if (DiagResult &  64) cout << "FORWARD LIMIT ACTIVATED\n";   //  64
  if (DiagResult & 128) cout << "REVERSE LIMIT ACTIVATED\n";   // 128
  if (DiagResult & 256) cout << "ENCODER ERROR\n";             // 256

  return;
}
/////////////////////////////////////////////////////////////////////////////

void ServoDemo(byte addr) {
byte group_address;

// set the gains:

//    KP = 100, KD = 1000, KI = 40, IL = 50, 
//    PWM Limit = 255, Current Limit = 0, Position Error Limit = 4000, Servo Rate = 1, 
//    Deadband Compensation = 0
      
      //ServoSetGain(addr, 100, 1000, 40, 50, 255, 0, 4000, 1, 0);
      ServoSetGain(addr, 20, 2000, 40, 50, 255, 0, 4000, 1, 0);
      
      TurnServoOn(addr); 

      FindIndex(addr);

// wait a second
      Sleep(1000);

      StartPositionMotion(addr, 6000, 50000, 1000, true);
      do {
        LdcnNoOp(addr);
        ReportError(addr, Diagnose(addr));  
      } while (!(LdcnGetStat(addr) & MOVE_DONE) && IsServoOn(addr));

      
// wait a second
      Sleep(1000);

// to start simultaneous motion:
//   - set a group address of modules
      group_address = 128;
      LdcnSetGroupAddr(addr,  group_address, true);
//    LdcnSetGroupAddr(addr2, group_address, false);
    
//   - Define motion for every module with parameter start_now = false:
      StartVelocityMotion(addr, 100000, 10000, false);
//    StartPositionMotion(addr2,500000, 300000, 10000, false); 
//   - start simultaneous motion:
      ServoStartMotion(group_address);

      Sleep(3000);

// Stop the motor at addr smoothly
      Servo_StopMotor(addr, STOP_SMOOTH);
// or stop all motors with group_address abruptly
      ServoStopMotor(group_address, SRV_ENABLE_AMP | STOP_ABRUPT);

// go back in PWM mode:
      StartPWMMotion(addr, -5, true);
      Sleep(1000); 

      ServoStopMotor(group_address, SRV_ENABLE_AMP | MOTOR_OFF);

//      TurnServoOn();

// go to position 0 
      GoToPosition(addr, 0, 200000, 20000);
}
/////////////////////////////////////////////////////////////////////////////