//---------------------------------------------------------------------------
#include "sio_util.h"
//---------------------------------------------------------------------------
typedef unsigned char byte;

typedef struct _LDCNMOD {
	byte	modtype;       	//module type
        byte	modver;	       	//module version number
        long    subver;         //additional version number
	short int statusitems;	//definition of items to be returned
	byte	stat;  	       	//status byte
        byte	groupaddr;     	//current group address
        bool	groupleader;	//TRUE if group leader
        void *	p;      	//pointer to specific module's data structure
} LDCNMOD;


#define MAXSIOERROR 3

//Define PIC baud rate divisors
#define	PB9600	       129
#define	PB19200		63
#define	PB38400	      0x81 // LS-132 only
#define	PB57600		20
#define	PB115200	10
#define	PB125000      0x27 // LS-132 only
#define	PB312500      0x0F // LS-132 only
#define	PB625000      0x07 // LS-132 only
#define	PB1250000     0x03 // LS-132 only

//Module type definitions:
#define	SERVOMODTYPE	 0
#define	SERVOHYBTYPE    90
#define SERVOANALOGTYPE 91
#define	IOMODTYPE  	 2
#define	STEPMODTYPE   	 3

//The following must be created for each new module type:
//		data structure XXXMOD
//		Initializer function NewXXXMod
//		Status reading function GetXXXStat
//		LdcnInit and SendLdcnCmd must be modified to include calls
//			to the two functions above

#define CKSUM_ERROR 	0x02	//Checksum error bit in status byte

#define MAXNUMMOD	33


//Initialization and shutdown
void InitVars(void);
void LdcnSetStopCtrl(byte addr, byte mode);

DLLENTRY(void) LdcnSetFastSerial( bool fast);
DLLENTRY(bool) LdcnGetFastSerial();

DLLENTRY(int)  LdcnInit(char *portname, unsigned int baudrate);
DLLENTRY(int)  LdcnHardResetBdr(char *portname, unsigned int baudrate);
DLLENTRY(int)  LdcnFullInit(char *portname, unsigned int baudrate);
DLLENTRY(int)  LdcnRestoreNetwork(char *portname, unsigned int baudrate);
DLLENTRY(int)  LdcnRestoreInitNetwork(char *portname, unsigned int baudrate);
DLLENTRY(int)  LdcnCheckFixedAddr(char *portname, unsigned int baudrate, byte maxaddr);
DLLENTRY(BOOL) LdcnSendCmd(byte addr, byte cmd, char *datastr, byte n);
DLLENTRY(void) FixSioError();
DLLENTRY(void) LdcnShutdown(void);

//Module type independant commands (supported by all module types)
DLLENTRY(BOOL) LdcnSetGroupAddr(byte addr, byte groupaddr, bool leader);
DLLENTRY(BOOL) LdcnDefineStatus(byte addr, byte statusitems);
DLLENTRY(BOOL) LdcnReadStatus(byte addr, byte statusitems);
DLLENTRY(BOOL) LdcnDefineStatus2(byte addr, short int statusitems);
DLLENTRY(BOOL) LdcnReadStatus2(byte addr, short int statusitems);
DLLENTRY(BOOL) LdcnChangeBaud(byte groupaddr, unsigned int baudrate);
DLLENTRY(BOOL) LdcnSynchInput(byte groupaddr);
DLLENTRY(BOOL) LdcnNoOp(byte addr);
DLLENTRY(BOOL) LdcnHardReset();
DLLENTRY(BOOL) LdcnResetDevice(byte addr);

//Retrieve module type independant data from a module's data structure
DLLENTRY(byte) LdcnGetStat     (byte addr);
DLLENTRY(byte) LdcnGetStatItems(byte addr);
DLLENTRY(short int) LdcnGetStatItems2(byte addr);
DLLENTRY(byte) LdcnGetModType  (byte addr);
DLLENTRY(byte) LdcnGetModVer   (byte addr);
DLLENTRY(long) LdcnGetModSubVer(byte addr);
DLLENTRY(byte) LdcnGetGroupAddr(byte addr);
DLLENTRY(BOOL) LdcnGroupLeader (byte addr);

DLLENTRY(int)    LdcnOpen(char *name, unsigned int baudrate);
DLLENTRY(HANDLE) GetComPort();
DLLENTRY(int)    GetSioError();
DLLENTRY(int)    IsBusy();

DLLENTRY(void) LdcnSetPacketCommMode(BOOL mode);
DLLENTRY(BOOL) LdcnSendCmdBuff();


// Read/Write EEPROM of DSP based controllers LS132 ,LS231, LS160
// EEPROM_addr should be between 0 and 63.

// read a 16-bit word from EEPROM
DLLENTRY(int)    ReadDSPEEPROM(byte addr, unsigned short int EEPROM_addr);
// result >= 0 - content of EEPROM
// result == -1 - invalid servo controller type
// result == -2 - invalid EEPROM_addr
// result == -3 - function not allowed when servo is on
// result == -4 - communication error
// result == -5 - communication not established yet

// write a 16-bit word to EEPROM
DLLENTRY(int)    WriteDSPEEPROM(byte addr, unsigned short int EEPROM_addr, unsigned short int Value);
// result >= 0 - OK
// result == -1 - invalid servo controller type
// result == -2 - invalid EEPROM_addr
// result == -3 - function not allowed when servo is on
// result == -4 - communication error
// result == -5 - communication not established yet

// recalculate and update the checksum
DLLENTRY(int)    UpdateDSPEEPROMChkSum(byte addr);
// result >= 0 - OK
// result == -1 - invalid servo controller type
// result == -2
// result == -3 - function not allowed when servo is on
// result == -4 - communication error
// result == -5 - communication not established yet

