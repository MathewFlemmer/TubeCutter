#include "stdafx.h"

#include "ldcncom.h"
#include "servo.h"
#include "path.h"
#include "ServoDemo.h"

#include <iostream>
#include <fstream>
using namespace std;

/////////////////////////////////////////////////////////////////////////////

void SetPathMode(byte grp_addr, byte axis_x, byte axis_y)
{
// Initialize LdcnLib data structure for contouring motion

// Set the group address of axes 
    LdcnSetGroupAddr(axis_x, grp_addr, true);    // Axis X is a group leader
    LdcnSetGroupAddr(axis_y, grp_addr, false);

// Initialize the servos
   ServoStopMotor(grp_addr, SRV_ENABLE_AMP | STOP_ABRUPT | ADV_MODE);

// Reset the current motor positions to zero
   ServoResetPos(grp_addr);  

// Set the required status items for the path control module
   LdcnDefineStatus(axis_x, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);
   LdcnDefineStatus(axis_y, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);
 

// Initialize path control module parameters
//       path frequency = 60 Hz
//       Store a minimum of 45 points in the path point buffer
//       X axis is module address axis_x
//       Y axis is module address axis_y
//       Z axis is not used (set its address to 0)
//       the group address is grp_addr
//           module axis_x is the group leader
//       Define measure units
//       X scale:  2000.0 counts = 1 unit
//       Y scale:  2000.0 counts = 1 unit
//       Z scale:     0.0 counts = 1 unit
//       acceleration = 5.0 units/second/second
//       deceleration = 2.0 units/second/second

   SetPathParams(P_60HZ, 45, axis_x, axis_y, 0, grp_addr, 2000, 2000, 0, 5, 2);

// set the origin to X = 0, Y = 0, Z = 0;
// feedrate = 10.0 units/second = maximum allowed speed
// and continuous path tangent tolerance = 10 degrees

   SetOrigin(0, 0, 0);
   SetFeedrate(10);          // Maximum velocity = 10 units/sec(10*2000 encoder counts/sec)
   SetTangentTolerance(10);  // set criterion for tangent segments - 10 degrees
}
/////////////////////////////////////////////////////////////////////////////

void ReportSegmentError(long res)
{
// report error message when building paths
    switch (res) {
      case -1: cout << "Segment added is not tangent to the previous segment.\n";
               break;
      case -2: cout << "Too many segments in the segment list.\n";
               break;
      case -3: cout << "Values given do not define a correct arc.\n";
               break;
      default: cout << "Undefined path segment error.\n";
    }
}
/////////////////////////////////////////////////////////////////////////////

void StartPathAndWaitUntilComplete(byte axis_x, byte axis_y)
{
// start a path movement and wait for it to finish before returning

long result;
byte status;

// Initialize the path but it doesn't start moving yet
   
    InitPath();
    
// Download path points to the SERVO CMC modules until all points are
// downloaded.  Motion will begin automatically when the minimum number
// of path points have been loaded.
    
    do {
        result = AddPathPoints();
     
        cout << "X: " << ServoGetPos(axis_x) << " Y: " << ServoGetPos(axis_y) << "\n";
    }
    while (result != -1);  // the path is done
    
// Poll the X axis module to detect when the path is complete
    
    do {
        LdcnNoOp(axis_x);               // retrieve current status data, loop
        LdcnNoOp(axis_y);               // until no longer in path mode

        cout << "X: " << ServoGetPos(axis_x) << " Y: " << ServoGetPos(axis_y) << "\n";

        status = ServoGetAux(axis_x);
    } while (status & PATH_MODE);
}
/////////////////////////////////////////////////////////////////////////////

void Diamond(byte axis_x, byte axis_y) 
{

    ServoResetPos(LdcnGetGroupAddr(axis_x));     // Reset current positions of all axes
    
    ClearSegList( 0, 0, 0);     // The motion will start at (0, 0, 0)

    AddLineSeg(20, 25, 0);

// If we try to add the next segment the function AddLineSeg() will return -1 (the
// line segment is not tangent to the previous segment)
// The criterion for tangent segments is defined by the SetTangentTolerance() function
// to 10 degrees
    StartPathAndWaitUntilComplete(axis_x, axis_y);

    SetFeedrate(3);           // Maximum velocity = 3 units/sec
// The current position is (20, 25).
// That's why the list of segments is initialized with (20, 25)
    ClearSegList(20, 25, 0);
// The end point of the segment is (40, 0)
    AddLineSeg(40, 0, 0);
    StartPathAndWaitUntilComplete(axis_x, axis_y);

    
    SetFeedrate(20);           // Maximum velocity = 20 units/sec
    ClearSegList(40, 0, 0);    // Starting new segment list at (40, 0)   
    AddLineSeg(20, -25, 0);    // and add a line to (20, -25)
    StartPathAndWaitUntilComplete(axis_x, axis_y);

    
    SetFeedrate(10);          // Maximum velocity = 10 units/sec
    ClearSegList(20, -25, 0);
    AddLineSeg(0, 0, 0);
    StartPathAndWaitUntilComplete(axis_x, axis_y);

}
/////////////////////////////////////////////////////////////////////////////

void ArcByLineSegments(byte axis_x, byte axis_y, int numSegments)
{
double angle;
double nextX, nextY, radius;
double pi;
long ii, result;

    SetTangentTolerance(10);

    ServoResetPos(LdcnGetGroupAddr(axis_x));
    ClearSegList(0, 0, 0);
    
    pi = 3.14159265358979;
    angle = -180;  // in degrees
    radius = 100;
    
// Note that the result line segments are tangent and it's possible to add all of them to
// only one segment list before its execution
// If 180 / numSegments < TangentTolerance the segments will not be tangent and
// this function will not work 
// if tangent tolerance == 10 numSegments should be at least 18
    
    for (ii = 1; ii <= numSegments; ii++) {
         angle = angle + 180 / numSegments;
         nextX = radius + radius * cos(angle * pi / 180);
         nextY = -radius * sin(angle * pi / 180);             // Calculate
         result = AddLineSeg(float(nextX), float(nextY), 0);  // and add next line segment
         if (result < 0) {
             ReportSegmentError(result);
             break;
         }
    }
    StartPathAndWaitUntilComplete(axis_x, axis_y);
}
/////////////////////////////////////////////////////////////////////////////


void ContinuousMotion(byte axis_x, byte axis_y) 
{
long result;
bool ok;
int cnt;

// The size of segments buffer is MAXSEG (= 2000) that is too mush for this example
// that's why I'll limit it to 20 to demonstrate continuous adding segments while 
// simultaneous execution is running
int max_buffer_segments = 20;


    SetFeedrate(10);

    ServoResetPos(LdcnGetGroupAddr(axis_x));

    ClearSegList(0, 0, 0);

    InitPath();

    cnt = 10;

    do {

        cnt--;          
        if (!cnt) break;  //


// if there is enough room for 5 segments in the buffer
        if (SegmentsInBuffer() < max_buffer_segments - 5) {
           result = AddLineSeg(-10, -10, 0);

// we should never receive result -2 (the buffer is full)
           if (result < 0) { ReportSegmentError(result); break; }
         
           result = AddArcSeg(-10, 10, 0, -20, 0, 0, 0, 0, -1);
           if (result < 0) { ReportSegmentError(result); break; }
        
           result = AddLineSeg(10, -10, 0);
           if (result < 0) { ReportSegmentError(result); break; }
        
           result = AddArcSeg(10, 10, 0, 20, 0, 0, 0, 0, 1);
           if (result < 0) { ReportSegmentError (result); break; }
    
           result = AddLineSeg(0, 0, 0);
           if (result < 0) { ReportSegmentError( result); break; }
        }

        while (SegmentsInBuffer() > max_buffer_segments / 2) { 

          result = AddPathPoints();   // send points execution to axes
          if (result == -2) { cout << "Error adding points\n"; break; }

          ok = IsServoOn(axis_x) && IsServoOn(axis_y) &&
              (ServoGetAux(axis_x) & PATH_MODE) && 
              (ServoGetAux(axis_y) & PATH_MODE);

          if (!ok) return; // something is wrong
        }
    } while (true);
}
/////////////////////////////////////////////////////////////////////////////

void PathDemo(byte axis_x, byte axis_y)
{

byte grp_addr = 128;

// set the gains:
//    KP = 100, KD = 1000, KI = 40, IL = 50, 
//    PWM Limit = 255, Current Limit = 0, Position Error Limit = 4000, Servo Rate = 1, 
//    Deadband Compensation = 0

    //ServoSetGain(axis_x, 100, 1000, 40, 50, 255, 0, 4000, 1, 0);
    //ServoSetGain(axis_y, 100, 1000, 40, 50, 255, 0, 4000, 1, 0);
      ServoSetGain(axis_x, 20, 2000, 40, 50, 255, 0, 4000, 1, 0);
      ServoSetGain(axis_y, 20, 2000, 40, 50, 255, 0, 4000, 1, 0);


// Initialize contouring motion
    SetPathMode(grp_addr, axis_x, axis_y);
    
    Diamond(axis_x, axis_y);

    ArcByLineSegments(axis_x, axis_y, 100);

    ContinuousMotion(axis_x, axis_y);

}
/////////////////////////////////////////////////////////////////////////////