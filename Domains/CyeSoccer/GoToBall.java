/*
 * GoToBall.java
 */

import	EDU.gatech.cc.is.util.*;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import  EDU.cmu.cs.coral.abstractrobot.*;
import java.util.*;

/**
 * <B>Introduction</B><BR>
 * Example of complex schema-based control system for a MultiForageN150 
 * robot.  It uses a motor schema-based configuration to search for 
 * attractor * objects and carry them to a homebase.  The configuration 
 * is built using Clay.  * It can be run in simulation or on a robot.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.4 $
 */

public class GoToBall  extends ControlSystemCye
	{

	public final static boolean DEBUG = true;
	private NodeVec2	steering_configuration;
	private NodeInt 	state_monitor;
	private i_FSA_ba STATE_MACHINE;
	  //	  private NodeVec2 MS_SWIRL_BALL;

	  private v_ResetableFixedPoint_ PS_WANDERPOINT_GLOBAL; // point we cirlce around
	  private NodeVec2 PS_WANDERPOINT; // robocentric
	  private v_ResetableFixedPoint_ PS_WAYPOINT_GLOBAL; // where we're going.
	  private NodeVec2 PS_WAYPOINT; // robocentric

	  private int _lastState; // records the last FSM state
	  private Random _random;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{

		//======
		// Set some initial hardware configurations.
		//======
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(0.65*abstract_robot.MAX_TRANSLATION);

		_lastState = -1;// set to a non-state;
		_random=new Random();
		
		//======
		// perceptual schemas
		//======
		//--- robot's global position

		NodeVec2 PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);
		NodeVec2 PS_GLOBAL_HEADING = new v_SteerHeading_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- homebase 
		NodeVec2      // the place to deliver
		PS_HOMEBASE0_GLOBAL = new v_FixedPoint_(4.9,0.0);
		NodeVec2      // make it egocentric
		PS_HOMEBASE0 = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE0_GLOBAL);

		//--- targets of visual class 3
		NodeVec2Array 
		PS_TARGETS0_EGO = 
			new va_VisualObjects_r(3,abstract_robot);
		NodeVec2Array 
		PS_TARGETS0_GLOBAL = 
			new va_Add_vav(PS_TARGETS0_EGO, PS_GLOBAL_POS);


		//--- get the closest one

		NodeVec2 PS_CLOSEST0 = new v_Closest_va(PS_TARGETS0_EGO);

		PS_WANDERPOINT_GLOBAL = new v_ResetableFixedPoint_(_random.nextDouble()*9.0-4.5,_random.nextDouble()*5.0-2.5);
		PS_WANDERPOINT = new v_GlobalToEgo_rv(abstract_robot,
				  PS_WANDERPOINT_GLOBAL);

		PS_WAYPOINT_GLOBAL = new v_ResetableFixedPoint_(_random.nextDouble()*9.0-4.5,_random.nextDouble()*5.0-2.5);
		PS_WAYPOINT = new v_GlobalToEgo_rv(abstract_robot,
						    PS_WAYPOINT_GLOBAL);


		//======
		// Perceptual Features
		//======
		
		// ball tracking stuff

		b_ResetableBoolean_ PF_BALL_TRACKED_OK = new b_ResetableBoolean_(false);
		b_Not_s PF_BALL_NOT_TRACKED_OK = new b_Not_s(PF_BALL_TRACKED_OK);
		b_ResetableBoolean_ PF_WAITING_TO_KICK_BALL = new b_ResetableBoolean_(false);
		b_ResetableBoolean_ PF_DONE_KICKING_BALL = new b_ResetableBoolean_(false);
		b_ResetableBoolean_ PF_BALL_FAR = new b_ResetableBoolean_(false);

		// is something visible?

		NodeBoolean PF_TARGET0_VISIBLE = new b_NonZero_v(PS_CLOSEST0);

		// is it not visible?
		NodeBoolean
		PF_NOT_TARGET0_VISIBLE = new b_Not_s(PF_TARGET0_VISIBLE);


		// close to homebase 
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE0 = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_HOMEBASE0_GLOBAL);

		// close to targetPoint
		NodeBoolean
		PF_CLOSE_TO_WAYPOINT = new b_Close_vv(0.5, PS_GLOBAL_POS,
						      PS_WAYPOINT_GLOBAL); // was 0.5

		//======
		// motor schemas
		//======
		// avoid obstacles
		// swirl around wanderpoint to waypoint
		NodeVec2
		MS_SWIRL_WANDERPOINT_WAYPOINT = new v_Swirl_vv(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_WANDERPOINT,
                        PS_WAYPOINT);

		NodeVec2
		MS_MOVE_TO_WAYPOINT = new v_LinearAttraction_v(2.0,0.0,PS_WAYPOINT);

		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(1.5,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// swirl obstacles wrt target 0
		NodeVec2
		MS_SWIRL_OBSTACLES_TARGET0 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST0);

		// swirl obstacles wrt homebase0
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE0 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_HOMEBASE0);
		NodeVec2
		MS_SWIRL_BALL = new v_SwirlToBall_v(PS_GLOBAL_POS, PS_CLOSEST0, 
						      PS_HOMEBASE0, PF_TARGET0_VISIBLE,-0.17,
                                                      PF_BALL_TRACKED_OK, PF_WAITING_TO_KICK_BALL,
						      PF_DONE_KICKING_BALL, PS_GLOBAL_HEADING);


		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(5,seed);

		// swirl obstacles wrt noise
		NodeVec2
		MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS,
			MS_NOISE_VECTOR);

/*
		v_StaticWeightedSum_va 
		AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0]  = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER.weights[1]  = 1.0;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER.weights[2]  = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;
*/


		v_StaticWeightedSum_va 
		AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0]  = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES;

		//				AS_WANDER.weights[3]  = 1.0;
		//				AS_WANDER.embedded[3] = MS_NOISE_VECTOR;

		AS_WANDER.weights[1]  = 2.0;
		AS_WANDER.embedded[1] = MS_MOVE_TO_WAYPOINT;

		AS_WANDER.weights[2]  = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;

		AS_WANDER.weights[3]  = 1.25;
		AS_WANDER.embedded[3] = MS_SWIRL_WANDERPOINT_WAYPOINT;


		//======
		// AS_GO_TO_TARGET0
		//======
		v_StaticWeightedSum_va 
		AS_ACQUIRE0 = new v_StaticWeightedSum_va();

		AS_ACQUIRE0.weights[0]  = 1.0; // 1.5
		AS_ACQUIRE0.embedded[0] = MS_AVOID_OBSTACLES;

		AS_ACQUIRE0.weights[1]  = 2.0;
		AS_ACQUIRE0.embedded[1] = MS_SWIRL_BALL;

		AS_ACQUIRE0.weights[2]  = 1.0; // 1.0
		AS_ACQUIRE0.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0;

		AS_ACQUIRE0.weights[3]  = 0.05; //0.05
		AS_ACQUIRE0.embedded[3] = MS_NOISE_VECTOR;

		v_StaticWeightedSum_va 
		AS_RESETWANDER = new v_StaticWeightedSum_va();

		AS_RESETWANDER.weights[0]  = 0.05; // was 0.05
		AS_RESETWANDER.embedded[0] = MS_NOISE_VECTOR;
		//======
		// STATE_MACHINE
		//======
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;
		
		// STATE 0 WANDER
		STATE_MACHINE.triggers[0][0]  = PF_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[0][0] = 1; // transition to ACQUIRE0
		STATE_MACHINE.triggers[0][1]  = PF_CLOSE_TO_WAYPOINT;
		STATE_MACHINE.follow_on[0][1] = 2; // transition to reset state

		// STATE 1 ACQUIRE0
		STATE_MACHINE.triggers[1][0] = PF_DONE_KICKING_BALL;
		STATE_MACHINE.follow_on[1][0] = 2; // transition to RESET_WANDER
		STATE_MACHINE.triggers[1][1] = PF_BALL_NOT_TRACKED_OK;
		STATE_MACHINE.follow_on[1][1] = 2; // transition to RESET_WANDER

		// STATE 2 RESET_WANDER
		STATE_MACHINE.triggers[2][0]  = PF_NOT_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[2][0] = 0; // go back to wander
		STATE_MACHINE.triggers[2][1]  = PF_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[2][1] = 1; // go back to wander

		state_monitor = STATE_MACHINE;


		//======
		// STEERING
		//======
		v_Select_vai
		STEERING = new v_Select_vai((NodeInt)STATE_MACHINE);

		STEERING.embedded[0] = AS_WANDER;
		STEERING.embedded[1] = AS_ACQUIRE0;
		STEERING.embedded[2] = AS_RESETWANDER;

		steering_configuration = STEERING;
		}
		
	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int takeStep()
		{
		Vec2 result;
		long	curr_time = abstract_robot.getTime();
		double speed;

		// STEER
		/*	result = MS_SWIRL_BALL.Value(curr_time);
		System.out.println("result " + result.t + " " + result.r);
		double angleToTurn = result.t;
		angleToTurn = Units.ClipRad(angleToTurn);
		System.out.println("angle to turn " + angleToTurn);
		abstract_robot.setSteerHeading(curr_time, angleToTurn);
		speed =  result.r;
		if (speed > 1.0)
		  speed = 1.0;
		abstract_robot.setSpeed(curr_time, speed);

		*/
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		if (result.r < 0.25 && result.r != 0.0)
		  result.r = 0.25;
		abstract_robot.setSpeed(curr_time, result.r);


		// STATE DISPLAY
		int state = STATE_MACHINE.Value(curr_time);
		if (state==2)
		  {
		    
		    PS_WANDERPOINT_GLOBAL.setPoint(_random.nextDouble()*9.0-4.5,_random.nextDouble()*5.0-2.5);
		    PS_WAYPOINT_GLOBAL.setPoint(_random.nextDouble()*9.0-4.5,_random.nextDouble()*5.0-2.5);

		    _lastState=state;
		  }
		if (state == 0)
			abstract_robot.setDisplayString("wander");
		else if (state == 1)
			abstract_robot.setDisplayString("go to ball");

		return(CSSTAT_OK);
		}
	}
