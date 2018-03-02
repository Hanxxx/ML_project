/*
 * ControlCye.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.cmu.cs.coral.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	java.lang.Math;

/**
 * <B>Introduction</B><BR>
 * The configuration 
 * is built using Clay.  * It can be run in simulation or on a robot.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1999, 2000 CMU
 *
 * @author Rosemary Emery
 * @version $Revision: 1.2 $
 */

public class ControlCye  extends ControlSystemCye
	{
	public final static boolean DEBUG = true;
	private NodeVec2	steering_configuration;
	double heading = 0.0;	
	double speed = 0.0;
	int counter = 0;

		
	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int takeStep()
		{
		long	curr_time = abstract_robot.getTime();

		// STEER
		if (curr_time == 0) 
			{
			heading = abstract_robot.getSteerHeading(curr_time);
			speed = 0.1;
			counter = 1;
			}

		else if (curr_time >= counter*30000) 
			{
			heading = heading + Math.PI/2;
			if (heading > 2*Math.PI)
				{
				heading -= 2*Math.PI;
				}
			speed = 0.1;
			counter++;
			}

		abstract_robot.setSteerHeading(curr_time, heading);
		abstract_robot.setSpeed(curr_time, speed);
		return 1;
		}
	}
