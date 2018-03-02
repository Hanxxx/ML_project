// This description file specifies the environment for a RoboCup 
// soccer game simulation in the JavaBotSim simulator.

//======
// SIMULATION BOUNDARY
//======
//
// bounds left right bottom top
//
// bounds statements set the bounds of the visible "playing field" in
// meters for a simulation.   If the aspect ratio of the bounds are not 
// the same as the graphical area set aside by the simulation, then 
// the robots may wander off the screen.  This will be fixed (eventually!)
// These are the bounds of a RoboCup field "base."  The actual field walls
// are a little bit smaller, but we reserve all this space so the complete
// field can be drawn

bounds -6 6 -4 4


//======
// SEED
//====== 
// 
// seed number
// 
// The seed statement sets the random number seed.  The default is
// -1

seed 10


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
// may be necessary.  Here, we run faster than real time:

time 1.0 // go 2x real time


//======
// TIMEOUT
//======
//
// timeout time
//
// The timeout statement indicates when the simulation will terminate in
// milliseconds.  The program automatically terminates when this time
// is reached.  If no timeout statement is given, the default is no
// termination.
//
timeout 600000 // ten seconds


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

maxtimestep 50 // 1/10th of a second


//======
// BACKGROUND COLOR
//======
//
// background color
//
// A background statement sets the background color for the simulation.
// The color must be given in hex format as "xRRGGBB" where RR indicates
// the red component (00 for none, FF for full), GG is the green component,
// and BB is the blue.  For soccer, we use dark green.

background xFFFFFF

//======
// WINDOWSIZE
//======
//
// windowsize width height
//
// The windowsize statement gives a default window size.  This can be
// overridden on the command line.

windowsize 511 350

//======
// OBJECTS
//======
//
// object objecttype x y theta forecolor backcolor visionclass
//
// Object statements cause a simulated object to be instantiated
// in the simulation.  Be sure to include the full class name for the
// object.  The x y and theta parameters set the initial position of 
// object in the field.  Forecolor and backcolor are the foreground
// and background colors of the object as drawn. The visionclass
// parameter is used to put each kind of object into it's own perceptual
// class.  That way when the simulated sensors of robots look for things
// they can be sorted by this identifier.
//

// SocFieldSmallSim is a special object that draws the field.  It
// the following are the field boundaries that the robots cannot go out of
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.5 2.525 4.5 2.525 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.5 -2.525 -4.5 -2.525 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.525 1.0 -4.525 2.5 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.525 -1.0 -4.525 -2.5 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.525 2.5 4.525 1.0 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.525 -2.5 4.525 -1.0 0.025 x00FF00 x000000 2
// the two goals
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.525 1.0 -4.925 1.0 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.925 1.0 -4.925 -1.0 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4.525 -1.0 -4.925 -1.0 0.025 x00FF00 x000000 2

linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.525 1.0 4.925 1.0 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.925 1.0 4.925 -1.0 0.025 x00FF00 x000000 2
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 4.525 -1.0 4.925 -1.0 0.025 x00FF00 x000000 2

// some random obstacles
//object EDU.gatech.cc.is.simulation.ObstacleSim -3.5 -2 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 0.5 -1 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 2.0 0 0.25 xC0C0C0 x000000 2
//object EDU.cmu.cs.coral.simulation.PolygonObstacleSim 2.0 0.0 0.7 0.3 xC0C0C0 x000000 2

// The ball
object CyeBallNoiseSim 0 0 0 0.1 xFF0000 x000000 3
//object CyeBallNoiseSim 0 0 0 0.2 x0000FF x000000 3
//object CyeBallNoiseSim 0 0 0 0.2 x00FF00 x000000 3

//======
// ROBOTS
//======
//
// robot robottype controlsystem x y theta forecolor backcolor 
//    visionclass
//
// Robot statements cause a robot with a control system to be instantiated
// in the simulation.  Be sure to include the full class name for the
// abstract robot type and your control system.  The y and theta
// parameters are actually ignored for soccer robots because they
// have pre-assigned initial locations.  The x parameter indicates
// whether the robot is on the east (positive) or west (negative).
// You can used different colors to tell your team or individual
// robots apart from one another.  The robots are assigned their player
// numbers according to the order in which they are listed here.
 robot EDU.cmu.cs.coral.abstractrobot.SimpleCyeSim GoToBall 0 0 0 x000000 x0000FF 0


