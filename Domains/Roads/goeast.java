/*
 * goeast.java
 */

import	java.io.*;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	EDU.gatech.cc.is.learning.*;

/**
 * <B>Introduction</B><BR>
 * Column formation robot.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class goeast  extends ControlSystemRescueVan
	{
	public final static boolean DEBUG = false;
	private NodeVec2	steering_configuration;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		//================
		// Set some initial hardware configurations.
		//================
		if (true) System.out.println("goeast .Configure()");
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(24.0);  //88 km/hour
		abstract_robot.setKinMaxRange(100);

		int tmp = abstract_robot.getPlayerNumber(-1);


		//================
		// perceptual schemas
		//================
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- goal direction
		NodeVec2  
		PS_GOAL_GLOBAL = new v_FixedPoint_(10000.0,0.0);
		NodeVec2  
		PS_GOAL = new v_GlobalToEgo_rv(abstract_robot, 
			PS_GOAL_GLOBAL);

		//--- starting position
		NodeVec2    
		PS_START_GLOBAL = new v_FixedPoint_(-25.0,0.0);
		NodeVec2
		PS_START = new v_GlobalToEgo_rv(abstract_robot, 
			PS_START_GLOBAL);

		//--- other robots
		NodeVec2Array // the locations of other robots
		PS_ROBOTS = new va_Teammates_r(abstract_robot);

		//--- unit center
		NodeVec2    
		PS_ZERO = new v_FixedPoint_(0.0,0.0);
		NodeVec2Array // locations of self and other robots
		PS_ROBOTS_AND_SELF = new va_Merge_vav(PS_ROBOTS,PS_ZERO);
		NodeVec2 // average the locations of other robots
		PS_UNIT_CENTER = new v_Average_va(PS_ROBOTS_AND_SELF);
		NodeVec2
		PS_UNIT_CENTER_GLOBAL = new v_EgoToGlobal_rv(abstract_robot, 
			PS_UNIT_CENTER);

		//================
		// motor schemas
		//================
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(9.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(5,seed);

		// swirl obstacles wrt goal
		NodeVec2
		MS_SWIRL_OBSTACLES_GOAL = new v_Swirl_vav(9,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_GOAL);

		// go to home
		NodeVec2
		MS_MOVE_TO_GOAL = new v_LinearAttraction_v(0.4,0.0,
			//PS_START);
			PS_GOAL);

		// go to unit center
		NodeVec2
		MS_MOVE_TO_UNIT_CENTER = new v_LinearAttraction_v(3.0,2.0,
			PS_UNIT_CENTER);

		double aso_gain = 0.0;
		double noise_gain = 0.0;
		double swirl_gain = 0.5;
		double mtg_gain = 0.7;
		double maintain_form_gain = 1.3;
		double unit_center_gain = 0.6;

		//================
		// AS_NO_FORMATION
		//================
		// go to the goal in no formation
		v_StaticWeightedSum_va 
		AS_NO_FORMATION = new v_StaticWeightedSum_va();

		AS_NO_FORMATION.weights[0]  = aso_gain;
		AS_NO_FORMATION.embedded[0] = MS_AVOID_OBSTACLES;

		AS_NO_FORMATION.weights[1]  = noise_gain;
		AS_NO_FORMATION.embedded[1] = MS_NOISE_VECTOR;

		AS_NO_FORMATION.weights[2]  = swirl_gain;
		AS_NO_FORMATION.embedded[2] = MS_SWIRL_OBSTACLES_GOAL;

		AS_NO_FORMATION.weights[3]  = mtg_gain;
		AS_NO_FORMATION.embedded[3] = MS_MOVE_TO_GOAL;

		//================
		// STEERING
		//================
		NodeVec2
		STEERING = AS_NO_FORMATION;

		//================
		// TURRET
		//================
		NodeVec2
                TURRET = STEERING;

		steering_configuration = STEERING;
		}
		
	private boolean done = false;
	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int takeStep()
		{
		Vec2	result;
		double	dresult;
		long	curr_time = abstract_robot.getTime();
		Vec2	p;

		if (done==false)
			{
			// STEER
			result = steering_configuration.Value(curr_time);
			abstract_robot.setSteerHeading(curr_time, result.t);
			abstract_robot.setSpeed(curr_time, 1.0);

			// DISPLAY the state
			//int state = state_monitor.Value(curr_time);
			String msg = "Driving East";
				
			abstract_robot.setDisplayString(msg);

			return(CSSTAT_OK);
			}
		return(CSSTAT_OK);
		}

	public void quit()
		{
		System.out.println("quit()");
		}
	public void trialInit()
		{
		System.out.println("trialInit()");
		}
	public void trialEnd()
		{
		System.out.println("trialEnd()");
                }
	}
