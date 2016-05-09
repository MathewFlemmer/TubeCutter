bool IsServoOn(byte addr);
bool TurnServoOn(byte addr);
void TurnServoOff(byte addr);
void Servo_StopMotor(byte addr, byte stop_mode);
bool StartHoming(byte addr, byte mode);
void FindIndex(byte addr);
void StartPositionMotion(byte addr, long position, long velocity, long acceleration, bool start_now);
void StartVelocityMotion(byte addr, long velocity, long acceleration, bool start_now);
void StartPWMMotion(byte addr, short int pwm, bool start_now);
void GoToPosition(byte addr, long position, long velocity, long acceleration);
void GetDeviceID(byte nummod);
int  Diagnose(byte addr);
void ReportError(byte addr, int DiagResult);
void ServoDemo(byte addr);