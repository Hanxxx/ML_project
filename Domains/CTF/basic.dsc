bounds -15 15 -10 10
time 100.0  // twice realtime
maxtimestep 200
timeout 3000000
//background xFFCC99
background xFFFFFF
windowsize 600 400

object EDU.gatech.cc.is.simulation.BinSim -10 0 0 0.50 xFF3333 x000000 3 
object EDU.gatech.cc.is.simulation.BinSim  10 0 0 0.50 x0066FF x000000 4 

// red robots
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	capturered  -10 2 0 xFF3333 x000000 2
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	capturered  -10 0 0 xFF3333 x000000 2
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	capturered  -10 -2 0 xFF3333 x000000 2

// blue robots
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	captureblue  10 2 0 x0066FF x000000 5 
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	harrassblue  10 0 0 x0066FF x000000 5 
robot EDU.gatech.cc.is.abstractrobot.CaptureSim 
	captureblue  10 -2 0 x0066FF x000000 5 


// obstacles
// lake
object EDU.gatech.cc.is.simulation.ObstacleSim -13  -11 0 5.0 x3333CC x000000 6

// trees
object EDU.gatech.cc.is.simulation.ObstacleSim 2.0 2.0 0 0.3 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim 8.0 -2.0 0 1.0 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim -13.0 -5.0 0 0.3
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim -10.0 5.0 0 0.3
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim 12.0 6.0 0 0.3
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim  8.0  -2.0 0 0.3
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim -14.0 5.0 0 0.3
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim -2.3 1.8 0 0.30 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim 2.3 -1.8 0 0.30 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim 5.0 1.0 0 0.30 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim -6 0.5 0 0.30 
	x669900 x000000 6
object EDU.gatech.cc.is.simulation.ObstacleSim 6.0 8 0 0.30 
	x669900 x000000 6

//object EDU.gatech.cc.is.simulation.ObstacleSim -4  4 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 4  -3.5 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim -3.5 -2 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
//object EDU.gatech.cc.is.simulation.ObstacleSim 4.0 4.0 0 0.25 xC0C0C0 x000000 2

// Red Flags
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	14.5 1.00 0 0.5 xFF3333 x000000 0
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	13.0 -2.0 0 0.5 xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	11.0 3.50 0 0.5 xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	10.5 -3.0 0 0.5 xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	14.5 -4.5 0 0.5 xFF3333 x000000 0 

object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	10.7 1.00 0 0.5    xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	10.0 2.70 0 0.5    xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	12.7 0.00 0 0.5    xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	14.0 -4.0 0 0.5    xFF3333 x000000 0 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	12 3.80 0 0.5    xFF3333 x000000 0 

//blue flags
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-14.5 1.00 0 0.5 x0066FF x000000 1
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-13.0 -2.0 0 0.5 x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-11.0 3.50 0 0.5 x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-10.5 -3.0 0 0.5 x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-14.5 -4.5 0 0.5 x0066FF x000000 1 

object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-10.7 1.00 0 0.5    x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-10.0 2.70 0 0.5    x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-12.7 0.00 0 0.5    x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-14.0 -4.0 0 0.5    x0066FF x000000 1 
object EDU.gatech.cc.is.simulation.AttractorFlagSim 
	-12 5.00 0 0.5    x0066FF x000000 1
