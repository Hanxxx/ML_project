bounds -5 15 -5 5

seed 3

time 10.0 

maxtimestep 100 

background xFFFFFF

object EDU.gatech.cc.is.simulation.BinSim  10 0 0 0.50 x0000BB x000000 4

robot  EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	GainsDemo 0 0 0 x000000 xFF0000 2

// obstacles
object EDU.gatech.cc.is.simulation.ObstacleSim 5 0 0 0.50 xC0C0C0 x000000 2
