windowsize 600 300

bounds -5 15 -5 5

seed 3

time 10.0 

maxtimestep 100 

background xFFFFFF

object EDU.gatech.cc.is.simulation.BinSim  10 0 0 0.50 x0000BB x000000 4

robot  EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim 
	SchemaDemo 0 0 0 x000000 xFF0000 2

// obstacles
object EDU.gatech.cc.is.simulation.ObstacleSim 10.0 1.0 0 0.30 xC0C0C0 x000000 2

// walls
linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4 1 5 1 0.2 xC0C0C0 x000000 2

linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim -4 -1 5 -1 0.2 xC0C0C0 x000000 2

//linearobject EDU.cmu.cs.coral.simulation.LinearObstacleSim 6 -1 8 1 0.2 xC0C0C0 x000000 2

object EDU.cmu.cs.coral.simulation.PolygonObstacleSim 7.0 -1.0 0.7 1.0 xC0C0C0 x000000 2
