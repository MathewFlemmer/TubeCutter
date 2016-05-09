#include "stdafx.h"
#include <iostream>
#include <fstream>
using namespace std;

#include "ldcncom.h"
#include "ServoDemo.h"
#include "StepperDemo.h"
#include "PathDemo.h"
#include "IODemo.h"

#include "servo.h"

byte Module_ID[MAXNUMMOD + 1];

struct ProfileGenerator {
	float StartPosition;
	float TargetPosition;
	float MaxVelocity;
	float Frequency;
	float Acceleration;
	float CurrentVelocity;

	short nPoints;
	short currentPoint;
	long pathBuffer[10000];
	short stepData[2];

	ProfileGenerator()
	{
		nPoints = 0;
		currentPoint = 0;
		StartPosition = 0;
		TargetPosition = 0;
		MaxVelocity = 2000;
		Frequency = 60; // motor driver executes 60 points per second
		Acceleration = 200;
		CurrentVelocity = 0;

	}
	int CalculatePath() //With 1 motor this is straightforwards, if there were multiple, the largest time taken could be calculated, and the others could be fit to smoothely take that much time.
	{
		int i;
		short nPoints, nAccelerationPoints, nDecelerationPoints, nConstantVelocityPoints;
		float timeToAccelerate, timeToDecelerate, distanceTravelledWhilstAccelerating, distanceTravelledWhilstDecelerating, timeAtMaxVelocity, distanceTravelledAtMaxVelocity;

		//we are going to have a start and end, and the acceleration will need to be applied and points generated.		
		timeToAccelerate = (MaxVelocity - CurrentVelocity)/Acceleration; // seconds
		timeToDecelerate = MaxVelocity/Acceleration; // seconds
		//this assumes there is time to accelerate and decelerate in the operation
		distanceTravelledWhilstAccelerating = CurrentVelocity * timeToAccelerate + 0.5 * Acceleration * pow(timeToAccelerate,2); // Units of distance		
		distanceTravelledWhilstDecelerating = MaxVelocity*timeToDecelerate - 0.5 * Acceleration * pow(timeToDecelerate,2);  // Units of distance
		timeAtMaxVelocity = (TargetPosition - StartPosition - distanceTravelledWhilstAccelerating - distanceTravelledWhilstDecelerating)/MaxVelocity;
		distanceTravelledAtMaxVelocity = MaxVelocity * timeAtMaxVelocity;
		nAccelerationPoints = timeToAccelerate * Frequency; // points
		nDecelerationPoints = timeToDecelerate * Frequency; // points
		nConstantVelocityPoints = timeAtMaxVelocity * Frequency; //points
		nPoints = nAccelerationPoints + nConstantVelocityPoints + nDecelerationPoints; //points
		//Iterate and calculate points arrived at during acceleration, constant velocity and deceleration sections.
		//Acceleration
		for(i = 0 ; i < nAccelerationPoints; i ++)
		{
			pathBuffer[i] = CurrentVelocity * i/Frequency + 0.5* Acceleration * pow((i/Frequency),2);
		}
		//ConstantVelocity
		for( i = 0; i < nConstantVelocityPoints; i++ )
		{
			pathBuffer[i + nAccelerationPoints] =  MaxVelocity * i/Frequency + distanceTravelledWhilstAccelerating;
		}
		//Deceleration
		for(i = 0; i < nDecelerationPoints; i ++)
		{
			pathBuffer[i + nAccelerationPoints + nConstantVelocityPoints] = MaxVelocity * i/Frequency - 0.5 * Acceleration * pow((i/Frequency),2);
		}
		stepData[0] = nPoints-1;
		stepData[1] = currentPoint;
		return nPoints;
	}
	int ProfileCompleted()
	{
		if (stepData[0] == stepData[1])
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	short GetNextPoint()
	{
		short returnValue;

		returnValue = pathBuffer[stepData[1]];
		stepData[1] += 1;

		return returnValue;
	}
	
};

int main(int argc, char* argv[])
{
	byte nummod;
	unsigned int baud_rate;
	byte addr = 2;
	byte sta, aux, npoints;
	int freq = 6;
	int nPoints;
	long points[ 7];
	long position1, position2;
	BOOL res;
	ProfileGenerator *MyProfile = new ProfileGenerator();

	LdcnDefineStatus (addr, SEND_POS|SEND_PERROR|SEND_NPOINTS|SEND_AUX);
	LdcnNoOp(addr);
	MyProfile->TargetPosition = 500000; //targetPos
	MyProfile->MaxVelocity = 10000; //get MaxVelocity;
	MyProfile->Acceleration = 1000; //getAcceleration;
	MyProfile->Frequency = 6; //pathPointsFrequency;
	MyProfile->CalculatePath();   
    baud_rate = 19200;
    nummod = LdcnInit("COM1", baud_rate);
    GetDeviceID(nummod);

    if (!nummod) {                      // no modules found
        cout << "No modules found\n";
        return 0; 
    }

	if ((LdcnGetModType( addr) == SERVOMODTYPE) && (LdcnGetModVer( addr) >= 20) && (LdcnGetModVer( addr) < 40)) {
		
		// set PID parameters !!! depend on the motor - motor 1
		ServoSetGain( 1, 20, 2000, 40, 50, 255, 0, 4000, 1, 0);
		ServoSetGain( 2, 20, 2000, 40, 50, 255, 0, 4000, 1, 0); //motor 2
      
		ServoResetPos( 1);
		ServoResetPos( 2);

		// turn on the servo
		TurnServoOn( 1); 
		TurnServoOn( 2); 

		// get some info from the drive. put a break point and check npoints
		LdcnDefineStatus(1,SEND_POS|SEND_VEL|SEND_AUX|SEND_NPOINTS);
		LdcnReadStatus( 1, SEND_POS | SEND_AUX | SEND_NPOINTS);
		sta = LdcnGetStat( 1);
		aux = ServoGetAux( 1);
		npoints = ServoGetNPoints( 1);

		LdcnDefineStatus(2,SEND_POS|SEND_VEL|SEND_AUX|SEND_NPOINTS);
		LdcnReadStatus( 2, SEND_POS | SEND_AUX | SEND_NPOINTS);
		sta = LdcnGetStat( 2);
		aux = ServoGetAux( 2);
		npoints = ServoGetNPoints( 2);
		// set path frequency in Hz
		ServoSetPathFreq( 1, freq);
		ServoSetPathFreq( 2, freq);
		//for (int i  = 0; i < 7; i++)
		//{
		//	points[ i] = MyProfile->GetNextPoint();
		//}

		points[ 0] = 0;
		points[ 1] = 1000;
		points[ 2] = 2000;
		points[ 3] = 3000;
		points[ 4] = 4000;
		points[ 5] = 5000;
		points[ 6] = 6000;

		res = ServoAddPathPoints( 1, 7, points, false);
		res = ServoAddPathPoints( 2, 7, points, false);
		// get some info from the drive. put a break point and check npoints - should be 7 now
		LdcnReadStatus( addr, SEND_POS | SEND_AUX | SEND_NPOINTS);
		sta = LdcnGetStat( addr);
		aux = ServoGetAux( addr);
		npoints = ServoGetNPoints( addr);
		//while(ServoGetNPoints(addr) < 45){
		//	if(ServoGetNPoints( addr) < 45){
				//for (int i  = 0; i < 7; i++)
				//{
				//	points[ i] = MyProfile->GetNextPoint();
				//}
		//		res = ServoAddPathPoints( addr, 7, points, false);
		//		LdcnReadStatus( addr, SEND_POS | SEND_AUX | SEND_NPOINTS);
		//		sta = LdcnGetStat( addr);
		//		aux = ServoGetAux( addr);
		//		npoints = ServoGetNPoints( addr);
		//	}
		//	else
		//		break;
		//}

		points[ 0] = 7000;
		points[ 1] = 8000;
		points[ 2] = 9000;
		points[ 3] = 10000;
		points[ 4] = 11000;
		points[ 5] = 12000;
		points[ 6] = 13000;
		
		int tracker = 14;
		res = ServoAddPathPoints( 1, 7, points, false);
		res = ServoAddPathPoints( 2, 7, points, false);
		// get some info from the drive. put a break point and check npoints - should be 14 now
		LdcnReadStatus( 1, SEND_POS | SEND_AUX | SEND_NPOINTS);
		sta = LdcnGetStat( 1);
		aux = ServoGetAux( 1);
		npoints = ServoGetNPoints( 1);

        cout << "Starting path point mode motion\n";

		ServoStartPathMode( 0xFF); //group addr

		do {
			//Sleep( 10); // 16 milliseconds per point. more now
			LdcnNoOp( 1);
			//ReportError( addr, Diagnose(addr));  		
			position1 = ServoGetPos(1);
			cout<<"position1:"<<position1<<"\n";
			LdcnNoOp( 2);
			//ReportError( addr, Diagnose(addr));  		
			position2 = ServoGetPos(2);
			cout<<"position2:"<<position2<<"\n";
			LdcnReadStatus( 1, SEND_POS | SEND_AUX | SEND_NPOINTS);
			npoints = ServoGetNPoints( 1);
			cout<<"points:"<<int(npoints)<<"\n";
			if(npoints < 30){
				for (int i = 0; i < 7; i++){
					points[ i] = 1000* tracker;//MyProfile->GetNextPoint();
					tracker = tracker + 1;
					if (tracker > 50)
						tracker = 50;

				}
				res = ServoAddPathPoints( 1, 7, points, false);
				res = ServoAddPathPoints( 2, 7, points, false);
			}

		} while (position1< 49900);//!MyProfile->ProfileCompleted());//!(LdcnGetStat( addr) & MOVE_DONE) && IsServoOn(addr));
		//while (!MyProfile->ProfileCompleted()){//!(LdcnGetStat( addr) & MOVE_DONE) && IsServoOn(addr));{
		//	if(ServoGetNPoints( addr) < 45){
		//		for(nPoints = 0; nPoints< 7; nPoints++){
		//			if(MyProfile->ProfileCompleted()){
		//				break; // path completed
		//			}
		//			else{
		//				points[nPoints] = MyProfile->GetNextPoint();
		//			}
		//		}
		//		res = ServoAddPathPoints( addr, nPoints, points, false);
		//		LdcnReadStatus( addr, SEND_POS | SEND_AUX | SEND_NPOINTS);
		//		sta = LdcnGetStat( addr);
		//		aux = ServoGetAux( addr);
		//		npoints = ServoGetNPoints( addr);
		//	}
		//}
        cout << "Path point mode motion completed\n";
		LdcnReadStatus( addr, SEND_POS | SEND_AUX | SEND_NPOINTS);
		sta = LdcnGetStat( addr);
		aux = ServoGetAux( addr);
		npoints = ServoGetNPoints( addr);
		int pos = ServoGetPos( addr);

		LdcnShutdown();
		return 0;
	}

    switch ( LdcnGetModType(1) ) {
      case SERVOMODTYPE: 
// if 2 drives for coordinated motion control are found at addresses 1 and 2
           if ((Module_ID[1] == 174) && (Module_ID[2] == 174)) 
                PathDemo(1, 2);  
           else ServoDemo(1); 
           break;
      case IOMODTYPE:  IODemo(1); break;
      case STEPMODTYPE:  StepperDemo(1); break;
    } 

    LdcnShutdown();
	return 0;
}
