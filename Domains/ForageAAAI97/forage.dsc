// This is an example description file for specifying the environment
// when using JavaBotSim.  


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

bounds -5 5 -5 5


//======
// TIME
//======
//
// time accel_rate
//
// The time statement sets the rate at which simulation time progresses
// with respect to real time.  "time 0.5" will cause the simulation to
// run at half speed, "time 1.0" will cause it to run at real time,
// while "time 4.0" will run at 4 times normal speed.  Be careful
// about too high of a value though because the simulation will
// lose fidelity.  In fact, for slow computers, values less than 1.0
// may be necessary to avoid jumpy behavior.  Here, we run at twice 
// real time:

time 2.0  // twice realtime


//======
// MAX TIME STEP
//======
//
// maxtimestep milliseconds
//
// maxtimestep statements set the maximum time (in milliseconds) that can
// transpire between discrete simulation steps.  This will keep the simulation
// from getting jumpy on slow machines, or when/if your process gets
// swapped out.

maxtimestep 100 // 1/10th of a second

//======
// WINDOWSIZE
//======
//
// windowsize width height
//
// The windowsize statement gives a default window size.  This can be
// overridden on the command line.

windowsize 500 500


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

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage 0 -1.5 0 x000000 xFF0000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	forage 0  1.5 0 x000000 xFF0000 2 


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

// simulation of bins
object EDU.gatech.cc.is.simulation.ObstacleSim 0  0.4 0 0.40 x0000FF x000000 4 
object EDU.gatech.cc.is.simulation.ObstacleSim 0 -0.4 0 0.40 xFFA000 x000000 3

// obstacles
object EDU.gatech.cc.is.simulation.ObstacleSim -2.0 -1.0 0 0.30 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 2.0 2.0 0 0.10 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 2.3 1.8 0 0.30 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim -4  4 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 4  -3.5 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim -3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 4.0 4.0 0 0.25 xC0C0C0 x000000 2

// attractors
object EDU.gatech.cc.is.simulation.AttractorSim 
	4.5 1.00 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	3.0 -2.0 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	1.0 3.50 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	0.5 -3.0 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	-4.5 -4.5 0 0.0762 x0000FF x000000 1 

object EDU.gatech.cc.is.simulation.AttractorSim 
	0.7 1.00 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	0.0 2.70 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	2.7 0.00 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	4.0 -4.0 0 0.0762 x0000FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorSim 
	-2 3.80 0 0.0762 x0000FF x000000 1 

object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 1.80 0 0.0762 xFFA000 x000000 0
object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 1.60 0 0.0762 xFFA000 x000000 0
object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 1.40 0 0.0762 xFFA000 x000000 0
object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 1.20 0 0.0762 xFFA000 x000000 0
object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 1.00 0 0.0762 xFFA000 x000000 0
object EDU.gatech.cc.is.simulation.SquiggleBallSim 
	0.9 0.80 0 0.0762 xFFA000 x000000 0
