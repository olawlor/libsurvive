//Data recorder mod with GUI showing light positions.
/*
 Modded by OSL 2017-04-16 to try to figure out calibration for RMC configuration:
  Emitters facing in the same direction, 1.5 meters apart.
 

*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <os_generic.h>
#include "src/survive_cal.h"
#include <CNFGFunctions.h>
#ifdef __unix__
#include <unistd.h>
#endif

#include "src/survive_config.h"


/**
  Return the current time in seconds (since something or other).
*/
#if defined(_WIN32)
#  include <sys/timeb.h>
#  define time_in_seconds_granularity 0.1 /* seconds */
double time_in_seconds(void) { /* This seems to give terrible resolution (60ms!) */
        struct _timeb t;
        _ftime(&t);
        return t.millitm*1.0e-3+t.time*1.0;
}
#else /* UNIX or other system */
#  include <sys/time.h> //For gettimeofday time implementation
#  define time_in_seconds_granularity 0.01 /* seconds */
double time_in_seconds(void) {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_usec*1.0e-6+tv.tv_sec*1.0;
}
#endif

#include "osl/vec4.h"


/* A coordinate frame made from three unit vectors */
typedef struct SurviveObjectOrientation {
  // Orientation matrix (orthogonal unit vectors)
  vec3 x,y,z;
  
} SurviveObjectOrientation;

// Initialize this orientation to the identity
void survive_orient_init(SurviveObjectOrientation *o) {
  o->x=vec3(1,0,0);
  o->y=vec3(0,1,0);
  o->z=vec3(0,0,1);
}

// Convert local coordinates out into global coordinates
vec3 survive_orient_global_from_local(SurviveObjectOrientation *o, vec3 local)
{
  return local.x*o->x + local.y*o->y + local.z*o->z;
}

// Convert global coordinates down into local coordinates
vec3 survive_orient_local_from_global(SurviveObjectOrientation *o, vec3 global)
{
  return vec3(dot(global,o->x), dot(global,o->y), dot(global,o->z));
}


// Apply these incremental rotation angles to this orientation
//    angle.x rotates about the x axis, etc.
//    angle vector is in local coordinates, radians, and right handed
void survive_orient_rotate(SurviveObjectOrientation *o, vec3 angle)
{
  vec3 oldZ=o->z;
  o->z+= angle.y*o->x - angle.x*o->y;
  o->y+=-angle.z*o->x;
	o->x=normalize(cross(o->y,o->z));
	o->y=normalize(cross(o->z,o->x));
	o->z=normalize(o->z);
}

// Nudge our coordinate frame to make these two vectors equal:
//    local_measured (local coordinates) is what it's reading
//    global_should is what it should read
//    strength is how fast it should get there (0-1)
void survive_orient_nudge(SurviveObjectOrientation *o, vec3 local_measured, vec3 global_should, FLT strength)
{
  local_measured=normalize(local_measured);
  vec3 local_should=normalize(survive_orient_local_from_global(o,global_should));
  vec3 shift=strength*(local_measured-local_should); // rotation force
  float max_shift=0.2; // maximum radian rotation to ever apply
  if (length(shift)>max_shift) shift=max_shift*normalize(shift);
  vec3 rotate=cross(shift,normalize(local_measured)); // torque vector
  
  // Rotate to apply that torque
  survive_orient_rotate(o,rotate);
}



/* The hardware configuration of a tracked sensor */
typedef struct SurviveSensorHardware {
  // The sensor's position, relative to the IMU
  vec3 position; 
  
  // The sensor's facing normal vector
  vec3 normal;
} SurviveSensorHardware;

/* Keeps track of the simulated position of the sensor */
typedef struct SurviveSensorTracker {
  
  // Angle, in radians, of last detection by this lighthouse.
  //   These are re-zeroed every integration step.
  double angle[NUM_LIGHTHOUSES][2];
  
} SurviveSensorTracker;

/* Simulates position of a tracked object */
typedef struct SurviveObjectSimulation {
  // time_in_seconds at our last integration step.  
  //    Used to compute timesteps.
  double last_integrate_time;
  
  // Position of the lighthouses in global coordinates
  //    FIXME: for multi-object tracking, this needs to be a shared pointer.
  vec3 lighthouse_position[NUM_LIGHTHOUSES];
  
  // Orientation of the lighthouses in global coordinates
  //    FIXME: for multi-object tracking, this needs to be a shared pointer.
  //   lighthouse 0 always has identity orientation.
  SurviveObjectOrientation lighthouse_orient[NUM_LIGHTHOUSES];
  
  // Position of our center in global coordinates (meters)
  vec3 position;
  
  // Velocity of our center in global coordinates (m/s)
  vec3 velocity;
  
  // Our orientation in global coordinates (orient.x points along our X axis)
  SurviveObjectOrientation orient;
  // IMU timesteps are faster:
  double last_imu_time;
  
  // Down vector, measured in local orient frame
  vec3 down;

  // Number of sensors actually present in these arrays:
  int nsensor;
  
  // Static info about the sensor hardware  
  const SurviveSensorHardware *hardware;
  
  // Dynamic info about the sensor tracking
  SurviveSensorTracker sensor[SENSORS_PER_OBJECT];
} SurviveObjectSimulation;


SurviveObjectSimulation *survive_sim_init(const char *type) {
  double now=time_in_seconds();
  SurviveObjectSimulation *o=(SurviveObjectSimulation *)malloc(sizeof(SurviveObjectSimulation));
  
  o->last_integrate_time=now;
  
  // HACK: hardcode lighthouse positions for robot setup.
  o->lighthouse_position[0]=vec3(0,0,0);
  o->lighthouse_position[1]=vec3(1.5,0,0);
  survive_orient_init(&o->lighthouse_orient[0]);
  survive_orient_init(&o->lighthouse_orient[1]);
  
  o->position=vec3(0,1.0,0);
  o->velocity=vec3(0,0,0);
  survive_orient_init(&o->orient);
  o->last_imu_time=now;
  o->down=vec3(0,0,0);
  
  const static SurviveSensorHardware hardware[]={
#include "survive_models/controller_points.h"
  };
  o->hardware=hardware;
  
  // Zero out the sensors:
  for (int S=0;S<SENSORS_PER_OBJECT;S++)
    for (int L=0;L<NUM_LIGHTHOUSES;L++)
      for (int A=0;A<2;A++)
        o->sensor[S].angle[L][A]=0.0;
  
  return o;
}

// Integrate the collected sensor sweep data
void survive_sim_integrate(SurviveObjectSimulation *o) {
  double now=time_in_seconds();
  double dt=now-o->last_integrate_time;
  o->last_integrate_time=now;
  
  // Update predicted position based on velocity
  vec3 nextP=o->position+dt*o->velocity;
  
  // Update predicted position based on sensor readings
  
  
  
}

void survive_vec3_print(const char *heading, const vec3 *v)
{
  printf("%s: ( %.5f %.5f %.5f )\n", heading, v->x, v->y, v->z);
}

// Dump important stuff to the screen:
void survive_sim_print(SurviveObjectSimulation *o)
{
  survive_vec3_print("position",&(o->position));
  survive_vec3_print("orient.x",&(o->orient.x));
  survive_vec3_print("orient.y",&(o->orient.y));
  survive_vec3_print("orient.z",&(o->orient.z));
  survive_vec3_print("down",&(o->down));
}

// Update simulation for these IMU readings
void survive_sim_imu(SurviveObjectSimulation *o, FLT * accelgyro)
{
  double now=time_in_seconds();
  double dt=now-o->last_imu_time;
  o->last_imu_time=now;
  vec3 accel=vec3(accelgyro[0], accelgyro[2], accelgyro[1]);
  accel*=1.0/4140.0; // emperical scale factor to 1.0g 
  o->down=accel;
  
  vec3 gyro=vec3( -accelgyro[3], -accelgyro[5], -accelgyro[4] );
  gyro*=1.0/1000.0; // units seem to be milliradians per second?
  gyro*=dt; // radians of rotation this frame
  survive_orient_rotate(&o->orient,gyro);
  
  // Keep the down vector pointing down
  survive_orient_nudge(&o->orient,
    o->down,vec3(0,0,-1.0), dt*1.0);
}







SurviveObjectSimulation *ww0=0;
struct SurviveContext * ctx;
const char *caldesc="Initializing...";
int  quit = 0;

void HandleKey( int keycode, int bDown )
{
	if( !bDown ) return;

	if( keycode == 'O' || keycode == 'o' )
	{
		survive_send_magic(ctx,1,0,0);
	}
	if( keycode == 'F' || keycode == 'f' )
	{
		survive_send_magic(ctx,0,0,0);
	}
	if( keycode == 'Q' || keycode == 'q' )
	{
		quit = 1;
	}
}

void HandleButton( int x, int y, int button, int bDown )
{
}

void HandleMotion( int x, int y, int mask )
{
}

void HandleDestroy()
{
}

//int bufferpts[32*2*3][2];
int bufferpts[32*3][4];


char buffermts[32*128*3];
unsigned int buffertimeto[32*3][4];

void my_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
//	if( timeinsweep < 0 ) return;
	survive_default_light_process( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	if( sensor_id < 0 ) return;
	if( acode < 0 ) return;
//return;
	int jumpoffset = sensor_id;
	//if( strcmp( so->codename, "WM0" ) == 0 || strcmp( so->codename, "WW0" ) == 0) jumpoffset += 32;
	//else if( strcmp( so->codename, "WM1" ) == 0 ) jumpoffset += 64;

/*
	if( acode % 2 == 0 && lh == 0) //data = 0
	{
		bufferpts[jumpoffset*2+0][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}
	if(  acode % 2 == 1 && lh == 0 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}


	if( acode % 2 == 0 && lh == 1 ) //data = 0
	{
		bufferpts[jumpoffset*2+0][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
	if( acode % 2 == 1 && lh == 1 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
	*/
	acode=(acode%2)+2*lh;
	bufferpts[jumpoffset][acode] = (timeinsweep-100000)/300;
	buffertimeto[jumpoffset][acode]=0;
}

int imu_updates=0;
FLT accelgyro[6];

void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro_new, uint32_t timecode, int id )
{
	survive_default_imu_process( so, mask, accelgyro, timecode, id );
	
	if (ww0) survive_sim_imu(ww0,accelgyro_new);
	imu_updates++;
	memcpy(accelgyro,accelgyro_new,6*sizeof(FLT));

	//if( so->codename[0] == 'H' )
	if( 0 )
	{
		printf( "I %s ( %f %f %f ) ( %f %f %f ) %08X %d\n", so->codename, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], timecode, id );
	}
}

/*

lh: lighthouse sending the pulse:
  0 is lighthouse B, for me the left.
  1 is lighthouse C, for me the right.

acode: angle code:
  0-1 for X-Y angles from lighthouse 0
  2-3 for X-Y angles from lighthouse 1

angle: radians

*/
FLT all_angles[32][4]; // sensor ID, then acode

void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
	
	/*
	printf("Angle:  %d  %d  %d  %.5f  %08X %.3g\n",
	  sensor_id, acode, lh, angle, timecode, length);
	*/
	acode=(acode%2)+2*lh;
	all_angles[sensor_id][acode]=angle;
}

#define MAX_SENSOR_NAME (3*32)
char* sensor_name[MAX_SENSOR_NAME];

void * GuiThread( void * v )
{
	short screenx, screeny;
	CNFGBGColor = 0x000000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Survive GUI Debug", 800, 600 );

	while(1)
	{
	  int sensor_id, acode;

#ifdef __unix__
usleep(20*1000); // limit to 50fps
#endif
		CNFGHandleInput();
		
		printf("\033[0;0f"); // seek to start of screen
		
	  if (ww0) {
	    survive_sim_print(ww0);
		}
		
		
		printf( "RAWIMU ( %7.1f %7.1f %7.1f ) ( %5.1f %5.1f %5.1f ) %d\n", accelgyro[0], accelgyro[2], accelgyro[1], -accelgyro[3], -accelgyro[5], -accelgyro[4], imu_updates);
		imu_updates=0;
		
	  for (sensor_id=0;sensor_id<32;sensor_id++)
	  {
	    printf("Sensor %2d: ",sensor_id);
	    for (acode=0;acode<4;acode++) 
	    {
	      float v=all_angles[sensor_id][acode];
	      if (buffertimeto[sensor_id][(acode/2)*2]<3) 
  	      printf("%8.4f  ",v);
  	    else
  	      printf("          ");
	    }
	    printf("\n");
	  }
		
		
		CNFGClearFrame();
		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		int i,nn;
		for( i = 0; i < 32*3; i++ )
		{
			for( nn = 0; nn < 4; nn+=2 )
			{
				if( buffertimeto[i][nn] < 5 ) // frames to draw old points
				{
					buffertimeto[i][nn]++;
					
					int pip=2;
					uint32_t color = i * 3231349;
					int x=bufferpts[i][nn];
					int y=bufferpts[i][nn+1];
					uint8_t r = 0xff;
					uint8_t g = 0xff;
					uint8_t b = 0xff;

					if (nn==0) { b = 0; g = 50; } //lighthouse B, red, master
					if (nn==2) r = 0; //lighthouse C, blue, slave

//					r = (r * (5-buffertimeto[i][nn])) / 5 ;
//					g = (g * (5-buffertimeto[i][nn])) / 5 ;
//					b = (b * (5-buffertimeto[i][nn])) / 5 ;
					CNFGColor( (b<<16) | (g<<8) | r );

					if (x == 0 || y==0) continue; //do not draw if aither coordinate is 0

					CNFGTackRectangle( x, y, x + pip, y + pip );
					CNFGPenX = x; CNFGPenY = y;
					CNFGDrawText( buffermts, 2 );

					if (i<MAX_SENSOR_NAME) {
						CNFGPenX = x+5; CNFGPenY = y+5;
						CNFGDrawText( sensor_name[i], 2 );
					}
				}
			}
		}

		CNFGColor( 0xffffff );
    /*
		char caldesc[256];
		survive_cal_get_status( ctx, caldesc, sizeof( caldesc ) );
		*/
		CNFGPenX = 3;
		CNFGPenY = 3;
		CNFGDrawText( caldesc, 4 );


		CNFGSwapBuffers();
		OGUSleep( 10000 );
	}
}




int main()
{
	ctx = survive_init( 0 );
	
	ww0=survive_sim_init("WW0");

	uint8_t i =0;
	for (i=0;i<MAX_SENSOR_NAME;++i) {
		sensor_name[i] = (char *)malloc(8);
		sprintf(sensor_name[i],"%d",i%32);
	}

	survive_install_light_fn( ctx,  my_light_process );
	survive_install_imu_fn( ctx,  my_imu_process );
	survive_install_angle_fn( ctx, my_angle_process );

	// survive_cal_install( ctx );

	OGCreateThread( GuiThread, 0 );
	

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0 && !quit)
	{
		//Do stuff.
	}

	survive_close( ctx );

	printf( "Returned\n" );
}


