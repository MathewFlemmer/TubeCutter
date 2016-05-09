//---------------------------------------------------------------------------
#define _WINDOWS
#if defined(_WINDOWS)
  #define DLLENTRY(type) extern "C" type __stdcall
  #include <windows.h>
#else
  #define DLLENTRY(type) extern "C" WINAPI __declspec(dllexport) type
  #include <vcl\vcl.h>
#endif
//---------------------------------------------------------------------------

DLLENTRY(int)    SimpleMsgBox(char *msgstr);
DLLENTRY(HANDLE) SioOpen(char *name, unsigned int baudrate);
DLLENTRY(BOOL)   SioChangeBaud(HANDLE ComPort, unsigned int baudrate);
DLLENTRY(BOOL)   SioPutChars(HANDLE ComPort, char *stuff, int n);
DLLENTRY(DWORD)  SioGetChars(HANDLE ComPort, char *stuff, int n);
         DWORD   SioGetChars10(HANDLE ComPort, char *stuff, int n);
DLLENTRY(DWORD)  SioTest(HANDLE ComPort);
DLLENTRY(BOOL)   SioClrInbuf(HANDLE ComPort);
DLLENTRY(BOOL)   SioClose(HANDLE ComPort);







