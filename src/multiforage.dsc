// This is an example description file for specifying the environment
// when using TBSim.  


//======
// SIMULATION BOUNDARY
//======
//
// bounds left right bottom top
//
// bounds statements set the bounds of the visible "playing field" in
// meters for a simulation.   If the aspect ratio of the bounds are not
// the same as the graphical area set aside by the simulation, then
// the robots may wander off the screen. 

bounds -8 8 -8 8


//======
// SEED
//======
//
// seed number
//
// The seed statement sets the random number seed.  The default is
// -1

seed 3


//======
// TIME
//======
//
// time accel_rate
//
// DEPRECATED.  This statement is ignored.

//======
// TIMEOUT
//======
//
// timeout time
//
// The timeout statement indicates when the simulation will terminate in
// milliseconds.  The program automatically terminates when this time
// is reached.  If no timeout statement is given, the default is no
// termination.  NOTE: you *must* use a timeout with a trials statement.
//
//timeout    936900 // ten seconds


//======
// TRIALS
//======
//
// trials num_trials
//
// The trials statement indicates that the simulation should be run
// a certain number of times.  Each trial automatically terminates when the
// timeout time is reached, then a new trial is begun.  Note: certain hooks
// are available in the ControlSystem class for you to know when trials
// begin and end.  See the javadoc documentation.
//
 trials 500 // 100 trials


//======
// MAX TIME STEP
//======
//
// maxtimestep milliseconds
//
// DEPRECATED.  This statement is processed as if it were a "timestep" statment.


//======
// TIMESTEP
//======
//
// timestep milliseconds
//
// timestep statements set the time (in milliseconds) transpices
// between discrete simulation steps.

timestep 100 // 1/10th of a second

//======
// WINDOWSIZE
//======
//
// windowsize width height
//
// The windowsize statement gives a default window size.  This can be
// overridden on the command line.

windowsize 800 800


//======
// BACKGROUND COLOR
//======
//
// background color
//
// A background statement sets the background color for the simulation.
// The color must be given in hex format as "xRRGGBB" where RR indicates
// the red component (00 for none, FF for full), GG is the green component,
// and BB is the blue.  Here we use white:

background xFFFFFF


//======
// ROBOTS
//======
//
// robot robottype controlsystem x y theta forecolor backcolor
//              visionclass
//
// robot statements cause a robot with a control system to be instantiated
// in the simulation.  Be sure to include the full class name for the
// abstract robot type and your control system.  The x y and theta
// parameters set the initial position of the robot in the field.
// You can used different colors to tell robots apart from one another.
// The visionclass indicates which color the robots see each other as.


//注意要修改个数的时候需要修改SquiggleBallSim.java中的元素个数

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage.multiforage0 7.5 7.5 -2.355 x000000 xFF0000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage.multiforage1 7.5  -7.5 2.355 x000000 xFF0000 2 

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage.multiforage2 -7.5  7.5 -0.785 x000000 xFF0000 2 
	
	robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage.multiforage3 -7.5 -7.5 0.785 x000000 xFF0000 2

//======
// OBJECTS
//======
//
// object objecttype x y theta radius forecolor backcolor visionclass
//
// Pbject statements instantiate things without control systems (like
// balls, bins, obstacles, etc. Be sure to include the full class name for the
// object.  The x y and theta parameters set the initial position of
// object in the field.  Forecolor and backcolor are the foreground
// and background colors of the object as drawn. The visionclass
// parameter is used to put each kind of object into it's own perceptual
// class.  That way when the simulated sensors of robots look for things
// they can be sorted by this identifier.

// simulation of bin
//object EDU.gatech.cc.is.simulation.ObstacleSim 0  0 0 0.10 xC0C0C0 x000000 4 

// obstacles
//object EDU.gatech.cc.is.simulation.ObstacleSim -4.5 -2.0 0 0.30 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 2.3 1.8 0 0.30 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim -4  4 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 4  -3.5 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 5 0 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim -3.5 -6 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 4.0 4.0 0 0.25 xC0C0C0 x000000 2


// attractors

object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0 5 0 0.15 xFFA000 x000000 0
//object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	//3.5 -1.0 0 0.15 xFFA000 x000000 0
//object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	//3.5 -3.40 0 0.0762 xFFA000 x000000 0
//object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	//-3.5 1.90 0 0.15 xFFA000 x000000 0
//object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	//-3.6 3.00 0 0.15 xFFA000 x000000 0
//object EDU.gatech.cc.is.simulation.SquiggleBallSim 
//	-3.5 -2.9 0 0.0762 xFFA000 x000000 0
	
