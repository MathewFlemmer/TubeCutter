bool IsMotorOn(byte addr);
void TurnMotorOn(byte addr);
void TurnMotorOff(byte addr);
bool SetParameters(byte addr, byte min_speed, byte run_current, byte hld_current, byte ADLimit,
                   byte speed_factor, bool ignore_limits, bool moff_on_stop, bool moff_on_limits);
void Stepper_StopMotor(byte addr, byte stop_mode);
bool CheckUpMotor(byte addr);
double TimeForDistance(byte addr, long goal_position, byte vel, byte acc);
void GoToPosition_Profiled(byte addr, long position, byte vel, byte acc);
void GoToPosition_Unprofiled(byte addr, long position, double step_period);
void StepperDemo(byte addr);