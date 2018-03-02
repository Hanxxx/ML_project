/*
 * SchemaDemo.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;


/**
 * Demonstrates navigation with a few motor schemas: move-to-goal,
 * noise, and avoid-obstacle.
 *
 * <A HREF="../../docs/copyright.html">Copyright</A>
 * (c)1999, 2000 Tucker Balch, Georgia Tech Research Corporation and CMU.
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */
public class SchemaDemo extends ControlSystemMFN150
	{
	public final static boolean DEBUG = true;
	private NodeVec2	turret_configuration;
	private NodeVec2	steering_configuration;
	private double mtggain = 1.0;
	private double oldmtggain = 1.0;
	private double avoidgain = 0.8;
	private double oldavoidgain = 0.8;
	private double noisegain = 0.2;
	private double oldnoisegain = 0.2;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{

	Vec2 a;
	Vec2 b;

	a = new Vec2(0,0);
	b = new Vec2(a);
	System.out.println("before "+b.x);
	a.setx(1);
	System.out.println("after  "+b.x);
	System.out.println("after  "+a.x);

		//======
		// Set some initial hardware configurations.
		//======
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(0.4*abstract_robot.MAX_TRANSLATION);

		//======
		// perceptual schemas
		//======
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- homebase 
		NodeVec2      // the goal location
		PS_HOMEBASE_GLOBAL = new v_FixedPoint_(10.0,0.0);
		NodeVec2      // make it egocentric
		PS_HOMEBASE = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE_GLOBAL);

		//======
		// motor schemas
		//======
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(3.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// move to goal
		NodeVec2
		MS_MOVE_TO_HOMEBASE = new v_LinearAttraction_v(0.4,0.0,PS_HOMEBASE);

		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(10,5);


		//======
		// AS_GO_HOME
		//======
		v_StaticWeightedSum_va 
		AS_GO_HOME = new v_StaticWeightedSum_va();

		AS_GO_HOME.weights[0]  = avoidgain;
		AS_GO_HOME.embedded[0] = MS_AVOID_OBSTACLES;

		AS_GO_HOME.weights[1]  = noisegain;
		AS_GO_HOME.embedded[1] = MS_NOISE_VECTOR;

		AS_GO_HOME.weights[2]  = mtggain;
		AS_GO_HOME.embedded[2] = MS_MOVE_TO_HOMEBASE;

		turret_configuration = AS_GO_HOME;
		steering_configuration = AS_GO_HOME;
		}
		
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

		// STEER
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, result.r);

		// TURRET
		result = turret_configuration.Value(curr_time);
		abstract_robot.setTurretHeading(curr_time, result.t);

		// STATE DISPLAY
		abstract_robot.setDisplayString("trying to go to the goal");

		return(CSSTAT_OK);
		}
	}
