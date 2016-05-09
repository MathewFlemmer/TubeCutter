#include "stdafx.h"

#include "ldcncom.h"
#include "stepper.h"

#include <iostream>
#include <fstream>
using namespace std;

/////////////////////////////////////////////////////////////////////////////

bool IsMotorOn(byte addr)
{
  LdcnNoOp(addr);  // get status byte
// and check up STP_AMP_ENABLED bit of status byte
  return (LdcnGetStat(addr) & STP_AMP_ENABLED); 
}
/////////////////////////////////////////////////////////////////////////////

void TurnMotorOn(byte addr)
{

  if (!IsMotorOn(addr)) {
    if ((LdcnGetStat(addr) & POWER_ON) == 0) {
        cout << "The Power On bit is set to 0\n";
        return;
    }

    LdcnReadStatus(addr, SEND_INBYTE);
    if ((StepGetCtrlMode(addr) & MOFF_STOP) && (StepGetInbyte(addr) & ESTOP)) {
        cout << "The motor is off when Stop is activated\n";
        return;
    }

    byte ADVal0 = StepGetAD(addr);
    byte ADVal1 = StepGetThermLimit(addr);

    if ((ADVal1 & 1) && (ADVal0 > ADVal1))
        cout << "A/D Limit is an odd number less than current A/D Value\n";

    if (!(ADVal1 & 1) && (ADVal0 < ADVal1))
        cout << "A/D Limit is an even number gteater than current A/D Value\n";

    StepStopMotor(addr, STOP_ABRUPT | STP_ENABLE_AMP);
  }
}
/////////////////////////////////////////////////////////////////////////////

void TurnMotorOff(byte addr)
{
  if (IsMotorOn(addr)) {
     StepStopMotor(addr, STOP_ABRUPT | STP_ENABLE_AMP);
     StepStopMotor(addr, STOP_ABRUPT);
   }
}
/////////////////////////////////////////////////////////////////////////////

bool SetParameters(byte addr, byte min_speed, byte run_current, byte hld_current, byte ADLimit,
                   byte speed_factor, bool ignore_limits, bool moff_on_stop, bool moff_on_limits)
{
byte mode;

// validate parameters values

   if (min_speed < 1 || min_speed > 250) {
       cout << "Minimum Velocity is out of range: 1 to 250\n";
       return false;
   }
   if (hld_current < 0 || hld_current > 200) {
       cout << "Holding Current is out of range: 0 to 200\n";
       return false;
   }
   if (ADLimit < 0 || ADLimit > 200) {
       cout << "A/D Limit is out of range: 0 to 200\n";
       return false;
   }
          
   if (hld_current > run_current) {
       cout << "The holding current must be less than the running current\n";
       return false;
   }

// Set operation parameters

  switch (speed_factor) {
    case 0: mode = SPEED_1X; break;
    case 1: mode = SPEED_2X; break;
    case 2: mode = SPEED_4X; break;
    case 3: mode = SPEED_8X; break;
    default:      
        cout << "The speed factor must be 1, 2 , 4 or 8\n";
        return false;
  }
 
  if (ignore_limits)  mode |= IGNORE_LIMITS;
  if (moff_on_stop)   mode |= MOFF_STOP;
  if (moff_on_limits) mode |= MOFF_LIMIT;

  return StepSetParam(addr, mode, min_speed, run_current, hld_current, ADLimit, 255);
}
/////////////////////////////////////////////////////////////////////////////

void Stepper_StopMotor(byte addr, byte stop_mode)
{

// save the current STP_ENABLE_AMP bit  

  byte mode = LdcnGetStat(addr) & STP_AMP_ENABLED ? STP_ENABLE_AMP : 0;

  switch (stop_mode) {
    case STOP_SMOOTH: mode = mode | STOP_SMOOTH; break;
    case STOP_ABRUPT: mode = mode | STOP_ABRUPT; break;
    default: return;
  }
  StepStopMotor(addr, mode);   // Send stop command
}
/////////////////////////////////////////////////////////////////////////////

bool CheckUpMotor(byte addr)
{

// read status byte and input byte from module
  LdcnReadStatus(addr, SEND_INBYTE);

  if (!(LdcnGetStat(addr) & STP_AMP_ENABLED)) {
     cout << "Motor is Off\n";
     return false;
  }
  
  if (StepGetInbyte(addr) & ESTOP) {
     cout << "Stop is activated\n";
     return false;
  }

  return true;
}

/////////////////////////////////////////////////////////////////////////////
// For profiled position mode:

double TimeForDistance(byte addr, long goal_position, byte vel, byte acc)
{
long current_pos, distance;
byte min_speed;
byte speed_mul;
double av_vel, acc_time, acc_distance;


// Distance( in Steps) = abs(goal_position - current position)
// speed * speed_multiplier = Speed in Steps/Sec
// (velocity - min_speed) * (64 - 0.25 * acceleration) = Time (milli Sec) for speed increment by 1


   LdcnReadStatus(addr, SEND_POS);
   current_pos = StepGetPos(addr);
   distance = abs(goal_position - current_pos);

   min_speed = StepGetMinSpeed(addr);

   switch (StepGetCtrlMode(addr) & 0x03) {  // the lowest two bits of the control byte
     case SPEED_8X: speed_mul = 200; break;
     case SPEED_4X: speed_mul = 100; break;
     case SPEED_2X: speed_mul =  50; break;
     case SPEED_1X: speed_mul =  25; break;
   }


   av_vel = ((double)vel + min_speed - 1 ) * speed_mul / 2000;    // Steps per micro second
   acc_time = (vel - min_speed) * (64 - 0.25 * acc);              // mSec
   acc_distance = av_vel * acc_time;                              // Steps 


   return vel * speed_mul == 0 ? 0 : 
                    (2 * acc_time)/1000 + (distance - 2 * acc_distance) / (vel * speed_mul);
}

/////////////////////////////////////////////////////////////////////////////
//Velocity Profile Mode

void GoToPosition_Profiled(byte addr, long position, byte vel, byte acc) 
{

  if (!CheckUpMotor(addr)) return;

  byte mode = START_NOW | LOAD_SPEED | LOAD_ACC | LOAD_POS;   // Position Mode

  if (vel < 0) {
      vel = -vel;
      mode = mode | STEP_REV;
  }

  if (vel < 1 || vel > 250) {
      cout << "Velocity is out of range.\n";
      return;
  }
  if (vel < StepGetMinSpeed(addr)) {
      cout << "Velocity is less than Min. Velocity.\n"; 
      return;
  }

  if (acc < 1 || acc > 255) {
      cout << "Acceleration is out of range.\n";
      return;
  }

  LdcnReadStatus(addr, SEND_POS);  // get current position

  if (abs(position - StepGetPos(addr)) > 0x7FFFFFFF) {
     cout << "The goal position should not differ from the current position by more then 0x7FFFFFFF.\n";
     return;
  }

  StepLoadTraj(addr, mode, position, vel, acc, 0);


// calculate time to reach the position
  
  double time2pos = TimeForDistance(addr, position, vel, acc); // in seconds
  cout << "Position will be reached after " << time2pos << " seconds.\n";


  do {
    LdcnNoOp(addr);
  } while (LdcnGetStat(addr) & MOTOR_MOVING);
}


/////////////////////////////////////////////////////////////////////////////
//Velocity Unrofiled Motion

// step_period - in steps per microsecond

void GoToPosition_Unprofiled(byte addr, long position, double step_period) 
{
byte speed_factor;


  byte mode = START_NOW | LOAD_ST | LOAD_POS; 

  LdcnReadStatus(addr, SEND_POS);  // get current position

  if (abs(position - StepGetPos(addr)) > 0x7FFFFFFF) {
     cout << "The goal position should not differ from the current position by more then 0x7FFFFFFF.\n";
     return;
  }

  if (step_period < 0) {
      step_period = -step_period;
      mode = mode | STEP_REV;
  }

  speed_factor = 0;

  switch (StepGetCtrlMode(addr) & 0x03) {  // the lowest two bits of the control byte
    case SPEED_8X: speed_factor = 3; break;
    case SPEED_4X: speed_factor = 2; break;
    case SPEED_2X: speed_factor = 1; break;
    case SPEED_1X: speed_factor = 0; break;
  }


// the range of step_period depends on chosen speed factor

  if (step_period < MaxStepPeriod(speed_factor) || step_period > MinStepPeriod(speed_factor)) {
     cout << "Step Period is out of range.\n";
     return;
  }


// calculate time to reach the position
  double time2pos = position/mSecPerStep2StepsPerSec(step_period); // in seconds

  cout << "Position will be reached after " << time2pos << " seconds.\n";

// The function StepLoadUnprofiledTraj() works similarly to StepLoadTraj() for 
// loading only unprofiled motion parameters.
// To avoid the inconvenience steptime parameter StepLoadUnprofiledTraj() uses step_period = the
// time for a single step in microseconds.
  StepLoadUnprofiledTraj(addr, mode, position, step_period);

/* If you prefer to use Step Time value, use this code:

   unsigned short int steptime;

// calculate Step Time value for step_period (in steps per microsecond)
   steptime = StepsPerSec2StepTime(mSecPerStep2StepsPerSec(step_period), speed_factor);

   StepLoadTraj(addr, mode, pos, 0, 0, steptime);
*/


  do {
     LdcnNoOp(addr);
  } while (LdcnGetStat(addr) & MOTOR_MOVING);

}
/////////////////////////////////////////////////////////////////////////////

void StepperDemo(byte addr) {

// set the operation parameters:
      SetParameters(addr, 1, 0, 0, 0, 0, false, false, true);
      
      TurnMotorOn(addr); 

// go to position 2000 with step period = 5000
      GoToPosition_Unprofiled(addr, 2000, 5000);


// go to position 0 with with velocity 4 (4 * 25 = 100 steps/second) and acceleration 1
      GoToPosition_Profiled(addr, 0, 4, 1) ;


      SetParameters(addr, 1, 0, 0, 0, 3, false, false, true);

// go to position 6000 with step period = 5000
      GoToPosition_Unprofiled(addr, 6000, 5000);

// go to position 0 with with velocity 4 (4 * 200 = 800 steps/second) and acceleration 1
      GoToPosition_Profiled(addr, 0, 4, 1) ;
}
/////////////////////////////////////////////////////////////////////////////