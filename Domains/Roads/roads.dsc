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

bounds -100 100 -50 50


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
// The time statement sets the rate at which simulation time progresses
// with respect to real time.  "time 0.5" will cause the simulation to
// run at half speed, "time 1.0" will cause it to run at real time,
// while "time 4.0" will run at 4 times normal speed.  Be careful
// about too high of a value though because the simulation will
// lose fidelity.  In fact, for slow computers, values less than 1.0
// may be necessary to avoid jumpy behavior.  Here, we try to run 
// at ten times real time.  In practicality, however, we are limited
// by the maxtimestep, below.

time 1.0  // one times realtime


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
// timeout 10000 // ten seconds


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
// trials 100 // 100 trials


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

windowsize 800 400


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

//background_image "/ur/trb/road/images/metro.jpg"


linearobject EDU.cmu.cs.coral.simulation.ImpassableTerrainSim 20 -100 50 -10 10.0 x0000B0 x000000 2
linearobject EDU.cmu.cs.coral.simulation.ImpassableTerrainSim 50  -10  30 100 10.0 x0000B0 x000000 2
linearobject EDU.cmu.cs.coral.simulation.RoadSim -80 20.0 -10 20.0 03 x00C000 x000000 2
linearobject EDU.cmu.cs.coral.simulation.SlowTerrainSim 10 20.0 80 20.0 03 xB0B000 x000000 2

linearobject EDU.cmu.cs.coral.simulation.SlowTerrainSim -80 -20 -10 -20.0 03 xB0B000 x000000 2
linearobject EDU.cmu.cs.coral.simulation.RoadSim 10 -20.0 80 -20 3 x00C000 x000000 2

robot EDU.gatech.cc.is.abstractrobot.RescueVanSim
	goeast -90 20.0 0 x000000 xFF0000 2 
robot EDU.gatech.cc.is.abstractrobot.RescueVanSim
	goeast -90 0.0 0 x000000 xFF0000 2 
robot EDU.gatech.cc.is.abstractrobot.RescueVanSim
	goeast -90 -20.0 0 x000000 xFF0000 2 

