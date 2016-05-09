#include "sio_util.h"
#include "math.h"
//---------------------------------------------------------------------------
// Defines:

// Segment types:
#define LINE 0
#define ARC 1

// Velocity profile type
#define VEL_TRAPEZ	 1
#define VEL_SCURVE	 2
#define VEL_INVALID	-1


// #define MAXSEG 2000     //Maximum number of segments
#define MAXSEG 1000        //Maximum number of segments

#define PI    (double) 3.14159265358979323846
#define TWOPI (double) 6.28318530717958647692
#define DTOR  (double) 0.01745329251994329577	// == 2*pi / 360.0

// Values for tangent tolerance
#define TAN_1DEGREE  (float) 0.999847695		// == cos(  1 degree)
#define TAN_3DEGREE  (float) 0.998629535		// == cos(  3 degree)
#define TAN_5DEGREE  (float) 0.996194698		// == cos(  5 degree)
#define TAN_10DEGREE (float) 0.984807753		// == cos( 10 degree)
#define TAN_20DEGREE (float) 0.939692208		// == cos( 20 degree)
#define TAN_45DEGREE (float) 0.707106781		// == cos( 45 degree)

//---------------------------------------------------------------------------
// Data types:

typedef float fp[3];    // floating point 3x1 vector

typedef long int ip[3]; // integer 3x1 vector

typedef struct {        // data type for line segments or arc segments
  byte type;            // LINE or ARC
  byte isTangent;		    // Is this segment tangent to previous one
  fp p1;                // Starting point
  fp p2;                // Ending point
  float len;            // Segment length
  fp c;                 // Center point (arcs only)
  fp norm;              // Normal vector (arcs only)
  float r;              // Radius (arcs only)
  float pathlen;		// Lenght of path up to the last tangent segment
  float maxvel;			// Maximum velocity (times VelOverride)
  float extvel;			// Velocity at which to leave current path
  int PathId;			// Single Id for all segment within a single tangent path
} segment;

typedef struct {        //data type for a coordinate frame
  fp	x;
  fp	y;
  fp	z;
  fp	p;
} frame;

//---------------------------------------------------------------------------
//Function prototypes:
float mag(fp p);
float dot(fp x, fp y);
void  cross(fp x, fp y, fp z);
float normalize(fp x, fp y);
void  fvmult(frame *F, fp x, fp y);
void  finvert(frame A, frame *B);
int   GetTanVect(segment *s, fp p, int endpoint);
void  GetArcFrame(segment *seg, frame *F);
void  GetLineSegPoint(segment *seg, float s, fp p);
int   GetNextPathpoint(long int *xp, long int *yp, long int *zp);

// Do NOT use InitPath() and SetOrigin() 
DLLENTRY(float) InitPath();
DLLENTRY(void) SetOrigin(float xoffset, float yoffset, float zoffset);

//Path mode API functions:
DLLENTRY(void) SetTangentTolerance(float theta);
DLLENTRY(int)  SetPathParams(int freq, int nbuf,
                             int xaxis, int yaxis, int zaxis, int groupaddr,
                             float xscale, float yscale, float zscale,
                             float accel, float decel);
DLLENTRY(void) SetFeedrate(float fr);

DLLENTRY(void) ClearSegList(float x, float y, float z);
DLLENTRY(int)  AddLineSeg(float x, float y, float z);
//Returns: position in segment list if OK
// -1 if segment is not tangent   -2 if segment list is full
DLLENTRY(int)  AddArcSeg(float x, float y, float z,        //end point
                         float cx, float cy, float cz,     //center point
                         float nx, float ny, float nz );   //normal vector
//Returns: position in segment list if OK;  -1 if segment is not tangent
//         -2 if segment list is full       -3 if arc data invalid

DLLENTRY(int)  AddPathPoints();
DLLENTRY(int)  SegmentsInBuffer();
DLLENTRY(int)  PathGetVelocity();
DLLENTRY(long) PathTime();
DLLENTRY(void) PathPauseMotion();
DLLENTRY(int)  PathResumeMotion();
DLLENTRY(BOOL) PathIsMotionPaused();

DLLENTRY(BOOL) PathSetOverride(long override);
DLLENTRY(long) PathGetOverride();

// Sets parameters (jerks) for S-Curved velocity profile
// Other parameters are taken from SetPathParams (accel, decel)
// Current speed, maximum speed and distance to go are obtained during calls to AddPathPoints
// Returns zero if jerks and accelerations are OK (i.e, not zero), or VEL_INVALID
DLLENTRY(int)  PathSetSProfileParams( float ajrka, float ajrkd, float djrka, float djrkd);
// Sets desired velocity profile. Returns it back, or VEL_INVALID
// By default, trapezoidal profile is executed
DLLENTRY(int)  PathSetProfileMode( int ProfileType);
DLLENTRY(int)  PathGetProfileMode( );
// Sets/returns various tolerances. Returns true on succes, false on failure
DLLENTRY(BOOL)  PathSetTolerances( float  length_tol, float  radius_tol, float  scurve_tol);
DLLENTRY(BOOL)  PathGetTolerances( float *length_tol, float *radius_tol, float *scurve_tol);
// Sets/returns velocity which to maintain at joints of non-tangent segments
// Same units as SetFeedrate
DLLENTRY(BOOL)  PathSetCornerVel( float exitvel);
DLLENTRY(float) PathGetCornerVel();
//---------------------------------------------------------------------------

