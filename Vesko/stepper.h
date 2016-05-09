//---------------------------------------------------------------------------
#include "sio_util.h"

//--------------------- Stepper Module specific stuff ---------------------------
typedef struct _STEPMOD {
    long		pos;     	//current position
    byte		ad;		//a/d value
    unsigned short int	st;          	//current step time
    byte		inbyte;         //input bits
    long		home;           //home position

//The following data is stored locally for reference
    long		cmdpos;			//last commanded position
    byte		cmdspeed;		//last commanded speed
    byte		cmdacc;			//last commanded acceleration
    short int	        cmdst;		        //last commanded step time
    double              st_period;
    byte                move_mode;              //last commanded move mode
    byte		min_speed;		//minimum running speed
    byte		stopctrl;		//stop control byte
    byte		outbyte;		//output bits
    byte		homectrl;		//homing control byte
    byte		ctrlmode;		//operating control mode byte
    byte		run_pwm;		//pwm for running current limit
    byte		hold_pwm;		//pwm for holding current limit
    byte		therm_limit;		//thermal limit
    byte                emergency_acc;
    byte                stat_io;                //IO byte, returned in status packet
    } STEPMOD;


//Step Module Command set:
#define	RESET_POS	  0x00	//Reset encoder counter to 0 (0 bytes)
#define	SET_ADDR	  0x01	//Set address and group address (2 bytes)
#define	DEF_STAT	  0x02	//Define status items to return (1 byte)
#define	READ_STAT	  0x03	//Read value of current status items
#define	LOAD_TRAJ  	  0x04	//Load trajectory data
#define START_MOVE	  0x05	//Start pre-loaded trajectory (0 bytes)
#define SET_PARAM	  0x06  //Set operating parameters (6 bytes)
#define	STOP_MOTOR 	  0x07	//Stop motor (1 byte)
#define	SET_OUTPUTS	  0x08	//Set output bits (1 byte)
#define SET_HOMING	  0x09  //Define homing mode (1 byte)
#define	SET_BAUD	  0x0A 	//Set the baud rate (1 byte)
#define RESERVED	  0x0B  //
#define SAVE_AS_HOME      0x0C	//Store the input bytes and timer val (0 bytes)
#define NOT_USED	  0x0D
#define	NOP		  0x0E	//No operation - returns prev. defined status (0 bytes)
#define HARD_RESET	  0x0F	//RESET - no status is returned

//Step Module STATUSITEMS bit definitions:
#define	SEND_POS	  0x01	//4 bytes data
#define	SEND_AD		  0x02	//1 byte
#define	SEND_ST		  0x04	//2 bytes
#define SEND_INBYTE	  0x08	//1 byte
#define SEND_HOME	  0x10	//4 bytes
#define SEND_ID		  0x20	//2 bytes
#define SEND_OUT          0x40  //1 byte

//Step Module LOAD_TRAJ control byte bit definitions:
#define	LOAD_POS	  0x01	//+4 bytes
#define LOAD_SPEED	  0x02	//+1 bytes
#define	LOAD_ACC	  0x04	//+1 bytes
#define LOAD_ST		  0x08	//+3 bytes
#define STEP_REV          0x10  //reverse dir
#define START_NOW	  0x80  //1 = start now, 0 = wait for START_MOVE command

//Step Module SET_PARAM operating mode byte bit definitions:
#define	SPEED_8X	  0x00	//use 8x timing
#define	SPEED_4X	  0x01	//use 4x timing
#define	SPEED_2X	  0x02	//use 2x timing
#define	SPEED_1X	  0x03	//use 1x timing
#define IGNORE_LIMITS     0x04	//Do not stop automatically on limit switches
#define MOFF_LIMIT        0x08  //Turn Off Motor on Limit
#define	MOFF_STOP         0x10  //Turn Off motor on Stop

//Step Module STOP_MOTOR control byte bit definitions:
#define	STP_ENABLE_AMP	  0x01	//1 = raise amp enable output, 0 = lower amp enable
#define STOP_ABRUPT       0x04	//set to stop motor immediately
#define STOP_SMOOTH	  0x08  //set to decellerate motor smoothly

//Step Module SET_HOMING control byte bit definitions:
#define ON_LIMIT1	  0x01	//home on change in limit 1
#define ON_LIMIT2	  0x02	//home on change in limit 2
#define HOME_MOTOR_OFF    0x04  //turn motor off when homed
#define ON_HOMESW	  0x08	//home on change in index
#define HOME_STOP_ABRUPT  0x10  //stop abruptly when homed
#define HOME_STOP_SMOOTH  0x20  //stop smoothly when homed

//Step Module Status byte bit definitions:
#define MOTOR_MOVING      0x01	//Set when motor is moving
#define CKSUM_ERROR	  0x02	//checksum error in received command
#define STP_AMP_ENABLED	  0x04	//set if amplifier is enabled
#define POWER_ON	  0x08	//set when motor power is on
#define AT_SPEED	  0x10  //set if the commanded velocity is reached.
#define VEL_MODE 	  0x20	//set when in velocity profile mode
#define TRAP_MODE	  0x40	//set when in trap. profile mode
#define HOME_IN_PROG      0x80  //set while searching for home, cleared when home found

//Step Module Input byte bit definitions:
#define	ESTOP		  0x01	//emergency stop input
#define	AUX_IN1		  0x02	//auxilliary input #1
#define	AUX_IN2		  0x04	//auxilliary input #2
#define FWD_LIMIT	  0x08	//forward limit switch
#define REV_LIMIT  	  0x10	//reverse limit switch
#define HOME_SWITCH       0x20	//homing limit switch

//Step module function prototypes:
STEPMOD * StepNewMod();

DLLENTRY(BOOL) StepGetStat(byte addr);
DLLENTRY(long) StepGetPos(byte addr);
DLLENTRY(byte) StepGetAD(byte addr);
DLLENTRY(unsigned short int) StepGetStepTime(byte addr);
DLLENTRY(byte) StepGetInbyte(byte addr);
DLLENTRY(long) StepGetHome(byte addr);
DLLENTRY(byte) StepGetIObyte(byte addr);
DLLENTRY(byte) StepGetMvMode(byte addr);
DLLENTRY(long) StepGetCmdPos(byte addr);
DLLENTRY(byte) StepGetCmdSpeed(byte addr);
DLLENTRY(byte) StepGetCmdAcc(byte addr);
DLLENTRY(unsigned short int) StepGetCmdST(byte addr);
DLLENTRY(double) StepGetStepPeriod(byte addr);

DLLENTRY(byte) StepGetCtrlMode(byte addr);
DLLENTRY(byte) StepGetMinSpeed(byte addr);
DLLENTRY(byte) StepGetRunCurrent(byte addr);
DLLENTRY(byte) StepGetHoldCurrent(byte addr);
DLLENTRY(byte) StepGetThermLimit(byte addr);
DLLENTRY(byte) StepGetEmAcc(byte addr);

DLLENTRY(byte) StepGetOutputs(byte addr);
DLLENTRY(byte) StepGetHomeCtrl(byte addr);
DLLENTRY(byte) StepGetStopCtrl(byte addr);

DLLENTRY(BOOL) StepSetParam(byte addr, byte mode,
	      		    byte minspeed, byte runcur, byte holdcur, byte thermlim, byte em_acc = 255);
DLLENTRY(BOOL) StepLoadTraj(byte addr, byte mode, long pos, byte speed, byte acc, unsigned short int steptime);
DLLENTRY(BOOL) StepLoadUnprofiledTraj(byte addr, byte mode, long pos, double step_period);
DLLENTRY(BOOL) StepResetPos(byte addr);
DLLENTRY(BOOL) StepStopMotor(byte addr, byte mode);
DLLENTRY(BOOL) StepSetOutputs(byte addr, byte outbyte);
DLLENTRY(BOOL) StepSetHoming(byte addr, byte mode);

DLLENTRY(int)    StepsPerSec2StepTime(double StepsPerSecond, int SpeedFactor);
DLLENTRY(double) StepTime2StepsPerSec(int InitialTimerCount, int SpeedFactor);
DLLENTRY(double) StepsPerSec2mSecPerStep(double StepsPerSecond);
DLLENTRY(double) mSecPerStep2StepsPerSec(double mSecPerStep);

DLLENTRY(double) MinStepPeriod(int SpeedFactor);
DLLENTRY(double) MaxStepPeriod(int SpeedFactor);
