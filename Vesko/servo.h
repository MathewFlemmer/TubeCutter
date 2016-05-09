//---------------------------------------------------------------------------
#include "sio_util.h"

//--------------------- Servo Module specific stuff ---------------------------
typedef	struct _GAINVECT {
    short int	kp;		//gain values
    short int	kd;
    short int	ki;
    short int	il;
    byte      	ol;
    byte      	cl;
    short int	el;
    byte      	sr;
    byte      	dc;
} GAINVECT;

typedef struct _SERVOMOD {
    long	pos;     	//current position
    long	pos2;     	//current position (motor encoder)
    short int	ad;		//a/d value
    short int	vel;            //current velocity
    byte	aux;            //auxilliary status byte
    long	home;           //home position
    short int	perror;		//position error
    short int	perror2;	//position error (motor encoder)
    byte	inport1;	//input port 1
    byte	inport2;	//input port 2
    byte	inport3;	//input port 3
//The following data is stored locally for reference
    long	cmdpos;		//last commanded position
    long	cmdvel;		//last commanded velocity
    long	cmdacc;		//last commanded acceleration
    byte	cmdpwm;	        //last commanded PWM value
    short int cmdadc;         //last commanded Ext Pos value // 10 bits for type 90 (SERVOHYBTYPE, ver. >= 2)
    GAINVECT    gain;
    long	stoppos;	//motor stop position (used by stop command)
    byte	outport1;	//output port val (used by I/O control)
    byte	outport2;	//output port val (used by I/O control)
    byte	outport3;	//output port val (used by I/O control)
    byte	outport4;	//output port val (used by I/O control)
    byte	stopctrl;	//stop control byte
    byte	movectrl;	//load_traj control byte
    byte	ioctrl;		//I/O control byte
    byte	homectrl;	//homing control byte
    byte	servoinit;	//set to 1 for servo on powerup, zero otherwise
    byte        stp_dir_mode;   // step/direction mode
    byte	ph_adv;		//phase advance (for ss-drive modules)
    byte	ph_off;		//phase offset (for ss-drive modules)
// LS-174
    byte        advmode;        //0 = LS-173, 1 = advanced mode
    byte        npoints;        //number of points in path buffer
    long        last_ppoint;    //last path point specified
// LS-132
    short int   outs132;        //16 digital outputs
    short int   PWMout132;      //PWM output value
    short int   freq132;        //# of ticks (50 uS) between points
    __int64     last_p132;      //last path point specified (with 8 bits fractional)
    short int   dinputs132;     //2 bytes digital inputs
    short int   ainputs132;     //2 bytes analog inputs
    byte        latchcnt;       //# of latched positions
    long        latchpos1;      //latched position 1
    long        latchpos2;      //latched position 1
    byte        npointsfrac;    //number of servo ticks for the last point
    short int   watchdog;       //2 bytes watchdog status
} SERVOMOD;


//Servo Module Command set:
#define	RESET_POS	  0x00	//Reset encoder counter to 0 (0 bytes)
#define	SET_ADDR	  0x01	//Set address and group address (2 bytes)
#define	DEF_STAT	  0x02	//Define status items to return (1 byte)
#define	READ_STAT	  0x03	//Read value of current status items
#define	LOAD_TRAJ  	  0x04	//Load trajectory date (1 - 14 bytes)
#define START_MOVE	  0x05	//Start pre-loaded trajectory (0 bytes)
#define SET_GAIN	  0x06  //Set servo gain and control parameters (13 or 14)
#define	STOP_MOTOR 	  0x07	//Stop motor (1 byte)
#define	IO_CTRL		  0x08	//Define bit directions and set output (1 byte)
#define SET_HOMING	  0x09  //Define homing mode (1 byte)
#define	SET_BAUD	  0x0A 	//Set the baud rate (1 byte)
#define CLEAR_BITS	  0x0B  //Save current pos. in home pos. register (0 bytes)
#define SAVE_AS_HOME      0x0C	//Store the input bytes and timer val (0 bytes)
#define EEPROM_CTRL	  0x0D  //Store or retrieve values from EEPROM
// LS-174
#define ADD_PATHPOINT     0x0D  //Adds path point in path mode
#define	NOP		  0x0E	//No operation - returns prev. defined status (0 bytes)
#define HARD_RESET	  0x0F	//RESET - no status is returned

// LS-174
//Servo Module RESET_POS control byte bit definitions:
//(if no control byte is used, reset is absolute)
#define REL_HOME 	  0x01	//Reset position relative to current home position

//Servo Module STATUSITEMS bit definitions:
#define	SEND_POS	  0x01	//4 bytes data
#define	SEND_AD		  0x02	//1 byte
#define	SEND_VEL	  0x04	//2 bytes
#define SEND_AUX	  0x08	//1 byte
#define SEND_HOME	  0x10	//4 bytes
#define SEND_ID		  0x20	//2 bytes
#define	SEND_PERROR	  0x40	//2 bytes
#define	SEND_INPORTS      0x80  //3 bytes
// LS-174
#define	SEND_NPOINTS      0x80  //1 byte
// LS-132
#define	SEND_DINP        0x0100  //2 bytes
#define	SEND_AINP        0x0200  //2 bytes
#define	SEND_MHOME       0x0400  //9 bytes
#define	SEND_NPOINTSFRAC 0x0800  //1 bytes
#define	SEND_WDOG        0x1000  //1 bytes // v.[20,29] or [33,39]
#define	SEND_POS2        0x2000  //4 bytes position   // v.[21,29] or [34,39]
                                 //+2 bytes pos error // v.[21,29] or [34,39]

//Servo Module LOAD_TRAJ control byte bit definitions:
#define	LOAD_POS	  0x01	//+4 bytes
#define LOAD_VEL	  0x02	//+4 bytes
#define	LOAD_ACC	  0x04	//+4 bytes
#define LOAD_PWM	  0x08	//+1 byte
// LS-139
#define ENC_VEL_MODE	  0x08	//Picoservo v.102+ 1 = encoder velocity mode
#define ENABLE_SERVO      0x10  //1 = servo mode, 0 = PWM mode
#define VEL_MODE	  0x20	//1 = velocity mode, 0 = trap. position mode
#define REVERSE		  0x40  //1 = command neg. PWM or vel, 0 = positive
// LS-174
#define MOVE_REL	  0x40  //1 = relative move, 0 = absolute move; works in advanced mode
#define START_NOW	  0x80  //1 = start now, 0 = wait for START_MOVE command

//Servo Module STOP_MOTOR control byte bit definitions:
#define	SRV_ENABLE_AMP	  0x01	//1 = raise amp enable output, 0 = lower amp enable
#define MOTOR_OFF	  0x02	//set to turn motor off
#define STOP_ABRUPT       0x04	//set to stop motor immediately
#define STOP_SMOOTH	  0x08  //set to decellerate motor smoothly
#define STOP_HERE	  0x10	//set to stop at position (4 add'l data bytes required)
// LS-174
#define ADV_MODE	  0x20	//Enables advanced mode
// LS-182AP type 90, ver >= 22
#define LOCK_DISABLE	  0x80	//Disables motions lock out on limits

//Servo Module IO_CTRL control byte bit definitions:
#define SET_OUT1	  0x01	//1 = set limit 1 output, 0 = clear limit 1 output
// LS-139
#define ENC_POWER_OFF	  0x01	//1 = turn encoder power off
#define SET_OUT2	  0x02	//1 = set limit 2 output, 0 = clear limit 2 output
#define IO1_IN		  0x04	//1 = limit 1 is an input, 0 = limit 1 is an output
#define IO2_IN		  0x08	//1 = limit 2 is an input, 0 = limit 2 is an output
#define WR_OUT1		  0x10  //Write to output port 1
#define WR_OUT2		  0x20  //Write to output port 2
#define WR_OUT3		  0x40  //Write to output port 3
// LS-174
#define FAST_PATH	  0x40	//0 = 30/60 Hz, 1 = 60/120 Hz
#define WR_OUT4		  0x80  //Write to output port 4

//Servo Module SET_HOMING control byte bit definitions:
#define ON_LIMIT1	  0x01	//home on change in limit 1
#define ON_LIMIT2	  0x02	//home on change in limit 2
#define HOME_MOTOR_OFF    0x04  //turn motor off when homed
#define ON_INDEX	  0x08	//home on change in index
#define HOME_STOP_ABRUPT  0x10  //stop abruptly when homed
#define HOME_STOP_SMOOTH  0x20  //stop smoothly when homed
#define ON_POS_ERR	  0x40	//home on excessive position error
#define	ON_CUR_ERR	  0x80  //home on overcurrent error

//Servo Module EEPROM_CTRL control byte bit definitions:
#define	STORE_GAINS       0x01	//Store servo gains
#define FETCH_GAINS	  0x02  //Retrieve stored gains
#define	STORE_VA          0x04	//Store velocity and acceleration
#define FETCH_VA	  0x08  //Retrieve stored velocity and acceleration
#define STORE_OUTPUTS     0x10  //Store output port values
#define FETCH_OUTPUTS     0x20  //Retrieve stored output port values
#define STORE_SI_BIT      0x40  //Store "servo initialize" bit
#define INIT_SERVO	  0x80  //Initializes servo on power-up

// LS-174
//Servo Module ADD_PATHPOINT frequency definitions
#define P_30HZ		  30	//30 hz path resolution
#define P_60HZ		  60    //60 hz path resolution
#define P_120HZ		  120   //120 hz path resolution

//Servo Module Status byte bit definitions:
#define MOVE_DONE	  0x01	//set when move done (trap. pos mode), when goal
                                //vel. has been reached (vel mode) or when not servoing
#define CKSUM_ERROR	  0x02	//checksum error in received command
#define OVERCURRENT	  0x04	//set on overcurrent condition (sticky bit)
#define POWER_ON	  0x08	//set when motor power is on
#define POS_ERR		  0x10	//set on excess pos. error (sticky bit)
#define LIMIT1		  0x20	//value of limit 1 input
#define LIMIT2		  0x40	//value of limit 2 input
#define HOME_IN_PROG      0x80  //set while searching for home, cleared when home found

//Servo Module Auxilliary status byte bit definitions:
#define INDEX		  0x01	//value of the encoder index signal
#define POS_WRAP	  0x02	//set when 32 bit position counter wraps around
     	     			//  (sticky bit)
#define SERVO_ON	  0x04	//set when position servo is operating
#define ACCEL_DONE	  0x08	//set when acceleration portion of a move is done
#define SLEW_DONE	  0x10  //set when slew portion of a move is done
#define SERVO_OVERRUN     0x20  //set if servo takes longer than the specified
     				//servo period to execute
// LS-174
#define PATH_MODE	  0x40  //path mode is enabled
// LS-182AP type 90 ver >= 22
#define MOTIONS_LOCKED	  0x80  //motion is locked out after homing on limits
// LS-1312
#define SET_LS132_OUTS    0x10  // set 2 bytes digital outputs
#define SET_LS132_PWM     0x20  // set 2 bytes PWM output
#define SET_LS132_FREQ    0x40  // set 2 bytes parh-point frequency

//Servo module function prototypes:
SERVOMOD * ServoNewMod();
DLLENTRY(BOOL) ServoGetStat(byte addr);
DLLENTRY(long) ServoGetPos(byte addr);
DLLENTRY(long) ServoGetPos2(byte addr);
DLLENTRY(byte) ServoGetAD(byte addr);
DLLENTRY(short int) ServoGetAD10(byte addr);
DLLENTRY(short int) ServoGetVel(byte addr);
DLLENTRY(byte) ServoGetAux(byte addr);
DLLENTRY(long) ServoGetHome(byte addr);
DLLENTRY(short int) ServoGetPError(byte addr);
DLLENTRY(short int) ServoGetPError2(byte addr);
DLLENTRY(byte) ServoGetInport1(byte addr);
DLLENTRY(byte) ServoGetInport2(byte addr);
DLLENTRY(byte) ServoGetInport3(byte addr);
DLLENTRY(long) ServoGetDigInputs(byte addr);
DLLENTRY(long) ServoGetAnInputs(byte addr);
DLLENTRY(byte) ServoGetOutport1(byte addr);
DLLENTRY(byte) ServoGetOutport2(byte addr);
DLLENTRY(byte) ServoGetOutport3(byte addr);
DLLENTRY(byte) ServoGetOutport4(byte addr);
DLLENTRY(long) ServoGetCmdPos(byte addr);
DLLENTRY(long) ServoGetCmdVel(byte addr);
DLLENTRY(long) ServoGetCmdAcc(byte addr);
DLLENTRY(byte) ServoGetCmdPwm (byte addr);
DLLENTRY(byte) ServoGetCmdAdc (byte addr);
DLLENTRY(short int) ServoGetCmdAdc10 (byte addr);
DLLENTRY(byte) ServoGetMoveCtrl(byte addr);
DLLENTRY(byte) ServoGetStopCtrl(byte addr);
DLLENTRY(byte) ServoGetHomeCtrl(byte addr);
DLLENTRY(byte) ServoGetIoCtrl(byte addr);
DLLENTRY(byte) ServoGetServoInit(byte addr);
DLLENTRY(byte) ServoGetSDMode(byte addr);
DLLENTRY(void) ServoGetGain(byte addr, short int * kp, short int * kd, short int * ki,
          short int * il, byte * ol, byte * cl, short int * el, byte * sr, byte * dc);
DLLENTRY(BOOL) ServoSetGain(byte addr, short int kp, short int kd, short int ki,
          short int il, byte ol, byte cl, short int el, byte sr, byte dc);
DLLENTRY(BOOL) ServoResetPos(byte addr);
DLLENTRY(BOOL) ServoClearBits(byte addr);
DLLENTRY(BOOL) ServoStopMotor(byte addr, byte mode);
DLLENTRY(BOOL) ServoLoadTraj(byte addr, byte mode, long pos, long vel, long acc, byte pwm);
DLLENTRY(BOOL) ServoLoadTraj10(byte addr, byte mode, long pos, long vel, long acc, short int pwm);
DLLENTRY(BOOL) ServoStartMotion(byte groupaddr);
DLLENTRY(BOOL) ServoSetHoming(byte addr, byte mode);
DLLENTRY(BOOL) ServoSetOutputs(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4);
DLLENTRY(BOOL) ServoEEPROMCtrl(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4);

// LS-174, LS-131
DLLENTRY(BOOL) ServoResetRelHome(byte addr);
DLLENTRY(BOOL) ServoSetFastPath(byte addr, bool fast);
DLLENTRY(void) ServoInitPath(byte addr);
DLLENTRY(BOOL) ServoAddPathPoints(byte addr, int npoints, long *path, bool high_freq);
DLLENTRY(BOOL) ServoStartPathMode(byte groupaddr);
DLLENTRY(byte) ServoGetNPoints(byte addr);

// LS-132
DLLENTRY(BOOL) ServoSetPathFreq(byte addr, int freq);
DLLENTRY(BOOL) ServoSetOutputs132(byte addr, byte mode, short int outs, short int PWM);
DLLENTRY(byte) ServoGetNMHomePos(byte addr);
DLLENTRY(long) ServoGetMHomePos1(byte addr);
DLLENTRY(long) ServoGetMHomePos2(byte addr);
DLLENTRY(byte) ServoGetNPointsFrac(byte addr);
DLLENTRY(short int) ServoGetOutputs132(byte addr);
DLLENTRY(short int) ServoGetPWMOutput132(byte addr);

