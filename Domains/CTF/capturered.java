/*
 * capturered.java
 */

import	java.io.*;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	EDU.gatech.cc.is.learning.*;

/**
 * <B>Introduction</B><BR>
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.2 $
 */

public class capturered  extends ControlSystemCapture
	{
	final int WANDER = 0;
	final int ACQUIRE_RED = 1;
	final int ACQUIRE_BLUE = 2;
	final int DELIVER_RED = 3;
	final int DELIVER_BLUE = 4;
	final int DELIVER = 5;
	final int DROP = 6;
	final int WANDER_IN_HOME_ZONE = 7; // not used

	final double HOME_ZONE = 2.0;

	public final static boolean DEBUG = false;
	private NodeVec2	turret_configuration;
	private NodeVec2	steering_configuration;
	private NodeDouble	gripper_fingers_configuration;
	private NodeDouble	gripper_height_configuration;
	private NodeDouble 	reward_monitor;
	private NodeInt 	state_monitor;
	private NodeInt 	action_monitor;

	private	String		filename_prefix;
	private	String		policy_filename;
	private	String		log_filename;
	private	String		profile_filename;
	private	int		num_delivered=0;



	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		//================
		// Set some initial hardware configurations.
		//================
		if (true) System.out.println("capturered .Configure()");
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(0.4*abstract_robot.MAX_TRANSLATION);
		abstract_robot.setGripperHeight(-1,0);   // put gripper down
		abstract_robot.setGripperFingers(-1,1);  // open gripper

		int tmp = abstract_robot.getPlayerNumber(-1);
		if (tmp<10) filename_prefix = "agent0";
		else filename_prefix = "agent";
		filename_prefix = filename_prefix.concat(
			String.valueOf(tmp));
		policy_filename = filename_prefix.concat(".policy");
		log_filename = filename_prefix.concat(".log");
		profile_filename = filename_prefix.concat(".profile");


		//================
		// perceptual schemas
		//================
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- HOMEBASE 
		NodeVec2      // the center of our little world
		PS_HOMEBASE_GLOBAL = new v_FixedPoint_(-10.0,0.0);
		NodeVec2      // make it egocentric
		PS_HOMEBASE = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE_GLOBAL);

		//--- WEST 
		NodeVec2      // the western front
		PS_WEST_GLOBAL = new v_FixedPoint_(-5,0.0);
		NodeVec2      // make it egocentric
		PS_WEST = new v_GlobalToEgo_rv(abstract_robot,
				PS_WEST_GLOBAL);

		//--- EAST 
		NodeVec2      // the eastern front
		PS_EAST_GLOBAL = new v_FixedPoint_(5,0.0);
		NodeVec2      // make it egocentric
		PS_EAST = new v_GlobalToEgo_rv(abstract_robot,
				PS_EAST_GLOBAL);

		//--- RED_BIN 
		NodeVec2      // the place to deliver red things
		PS_RED_BIN_GLOBAL = new v_FixedPoint_(-10,0.0);
		NodeVec2      // make it egocentric
		PS_RED_BIN = new v_GlobalToEgo_rv(abstract_robot,
				PS_RED_BIN_GLOBAL);

		//--- BLUE_BIN 
		NodeVec2      // the place to deliver blue things
		PS_BLUE_BIN_GLOBAL = new v_FixedPoint_(0.5,0.0);
		NodeVec2      // make it egocentric
		PS_BLUE_BIN = new v_GlobalToEgo_rv(abstract_robot,
				PS_BLUE_BIN_GLOBAL);

		//=== RED TARGETS (visual class 0)
		NodeVec2Array 
		PS_RED_TARGETS_EGO = 
			new va_VisualObjects_r(0,abstract_robot);
		NodeVec2Array 
		PS_RED_TARGETS_GLOBAL = 
			new va_Add_vav(PS_RED_TARGETS_EGO, PS_GLOBAL_POS);

		//--- filter out red targs in bins
		NodeVec2Array 
		PS_RED_TARGETS_GLOBAL_FILT = new va_FilterOutClose_vva(1.0,
			PS_HOMEBASE_GLOBAL, PS_RED_TARGETS_GLOBAL);

		//--- make them egocentric
		NodeVec2Array 
		PS_RED_TARGETS_EGO_FILT = new va_Subtract_vav(
			PS_RED_TARGETS_GLOBAL_FILT, PS_GLOBAL_POS);

		//--- get the closest one
		NodeVec2
		PS_CLOSEST_RED = new v_Closest_va(PS_RED_TARGETS_EGO_FILT);

		//--- filter out red targs in home zone
		NodeVec2Array 
		PS_RED_TARGETS_OUT_HZ_GLOBAL_FILT = new va_FilterOutClose_vva(
			HOME_ZONE+.5,
			PS_HOMEBASE_GLOBAL, 
			PS_RED_TARGETS_GLOBAL);

		//--- make them egocentric
		NodeVec2Array 
		PS_RED_TARGETS_OUT_HZ_EGO_FILT = new va_Subtract_vav(
			PS_RED_TARGETS_OUT_HZ_GLOBAL_FILT, PS_GLOBAL_POS);

		//--- get the closest one
		NodeVec2
		PS_CLOSEST_RED_OUT_HZ = new v_Closest_va(
			PS_RED_TARGETS_OUT_HZ_EGO_FILT);


		//=== BLUE TARGETS (visual class 1)
		NodeVec2Array 
		PS_BLUE_TARGETS_EGO = 
			new va_VisualObjects_r(1,abstract_robot);
		NodeVec2Array 
		PS_BLUE_TARGETS_GLOBAL = 
			new va_Add_vav(PS_BLUE_TARGETS_EGO, PS_GLOBAL_POS);

		//--- filter out targets in bins
		NodeVec2Array 
		PS_BLUE_TARGETS_GLOBAL_FILT = new va_FilterOutClose_vva(1.0,
			PS_HOMEBASE_GLOBAL, PS_BLUE_TARGETS_GLOBAL);

		//--- make them egocentric
		NodeVec2Array 
		PS_BLUE_TARGETS_EGO_FILT = new va_Subtract_vav(
			PS_BLUE_TARGETS_GLOBAL_FILT, PS_GLOBAL_POS);

		//--- get the closest one
		NodeVec2
		PS_CLOSEST_BLUE = new v_Closest_va(PS_BLUE_TARGETS_EGO_FILT);

		//--- filter out blue targs in home zone
		NodeVec2Array 
		PS_BLUE_TARGETS_OUT_HZ_GLOBAL_FILT = new va_FilterOutClose_vva(
			HOME_ZONE+.5,
			PS_HOMEBASE_GLOBAL, 
			PS_BLUE_TARGETS_GLOBAL);

		//--- make them egocentric
		NodeVec2Array 
		PS_BLUE_TARGETS_OUT_HZ_EGO_FILT = new va_Subtract_vav(
			PS_BLUE_TARGETS_OUT_HZ_GLOBAL_FILT, PS_GLOBAL_POS);

		//--- get the closest one
		NodeVec2
		PS_CLOSEST_BLUE_OUT_HZ = new v_Closest_va(
			PS_BLUE_TARGETS_OUT_HZ_EGO_FILT);

		//=== type of object in the gripper
		NodeInt
		PS_IN_GRIPPER = new i_InGripper_r(abstract_robot);


		//================
		// Perceptual Features
		//================
		// is a red visible?
		NodeBoolean
		PF_RED_VISIBLE = new b_NonZero_v(PS_CLOSEST_RED);

		// is a red not visible?
		NodeBoolean
		PF_NOT_RED_VISIBLE = new b_Not_s(PF_RED_VISIBLE);

		// is a blue visible?
		NodeBoolean
		PF_BLUE_VISIBLE = new b_NonZero_v(PS_CLOSEST_BLUE);

		// is a blue not visible?
		NodeBoolean
		PF_NOT_BLUE_VISIBLE = new b_Not_s(PF_BLUE_VISIBLE);

		// is a red visible outside HZ?
		NodeBoolean
		PF_RED_VISIBLE_OUT_HZ = new b_NonZero_v(PS_CLOSEST_RED_OUT_HZ);

		// is a red not visible outside HZ?
		NodeBoolean
		PF_NOT_RED_VISIBLE_OUT_HZ = new b_Not_s(PF_RED_VISIBLE_OUT_HZ);

		// is a blue visible outside HZ?
		NodeBoolean
		PF_BLUE_VISIBLE_OUT_HZ = new b_NonZero_v(
			PS_CLOSEST_BLUE_OUT_HZ);

		// is a blue not visible outside HZ?
		NodeBoolean
		PF_NOT_BLUE_VISIBLE_OUT_HZ = new b_Not_s(
			PF_BLUE_VISIBLE_OUT_HZ);

		// is a red thing in the gripper?
		NodeBoolean
		PF_RED_IN_GRIPPER = new b_Equal_i(0,PS_IN_GRIPPER);

		// is a blue thing in the gripper?
		NodeBoolean
		PF_BLUE_IN_GRIPPER = new b_Equal_i(1,PS_IN_GRIPPER);

		// close to homebase 
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE = new b_Close_vv(HOME_ZONE, PS_GLOBAL_POS,
			PS_HOMEBASE_GLOBAL);

		// not close to homebase
		NodeBoolean
		PF_NOT_CLOSE_TO_HOMEBASE = new b_Not_s(PF_CLOSE_TO_HOMEBASE);

		// for drop criteria
		NodeBoolean
		PF_REAL_CLOSE_TO_HOMEBASE = new b_Close_vv(1.0, PS_GLOBAL_POS,
			PS_HOMEBASE_GLOBAL);

		// drop criteria
		NodeBoolean
		PF_DROP_CRITERIA = new b_Not_s(PF_CLOSE_TO_HOMEBASE);

		// close to red bin 
		NodeBoolean
		PF_CLOSE_TO_RED_BIN = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_RED_BIN_GLOBAL);

		// close to blue bin 
		NodeBoolean
		PF_CLOSE_TO_BLUE_BIN = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_BLUE_BIN_GLOBAL);


		//================
		// motor schemas
		//================
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(1.5,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(5,seed);

		// avoid the homebase
		NodeVec2
		MS_AVOID_HOMEBASE = new v_Avoid_v(1.0,
			abstract_robot.RADIUS + 0.1,
			PS_HOMEBASE);

		// avoid the HOME_ZONE
		NodeVec2
		MS_AVOID_HOME_ZONE = new v_Avoid_v(4.0,
			abstract_robot.RADIUS + 0.1,
			PS_HOMEBASE);

		// stay in the HOME_ZONE
		NodeVec2
		MS_STAY_IN_HOME_ZONE = new v_LinearAttraction_v(
			HOME_ZONE,
			HOME_ZONE-1, 
			PS_HOMEBASE);

		// stay in the east territory
		NodeVec2
		MS_STAY_IN_EAST_TERRITORY = new v_LinearAttraction_v(5.0,
			4.0, PS_EAST);

		// stay in the west territory
		NodeVec2
		MS_STAY_IN_WEST_TERRITORY = new v_LinearAttraction_v(5.0,
			4.0, 
			PS_WEST);

		// avoid the red bin
		NodeVec2
		MS_AVOID_RED_BIN = new v_Avoid_v(
			2.0,
			abstract_robot.RADIUS + 0.1,
			PS_RED_BIN);

		// avoid the blue bin
		NodeVec2
		MS_AVOID_BLUE_BIN = new v_Avoid_v(
			2.0,
			abstract_robot.RADIUS + 0.1,
			PS_BLUE_BIN);

		// swirl obstacles wrt red targets
		NodeVec2
		MS_SWIRL_OBSTACLES_RED = new v_Swirl_vav(
			2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST_RED);

		// swirl obstacles wrt blue targets
		NodeVec2
		MS_SWIRL_OBSTACLES_BLUE = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST_BLUE);

		// swirl obstacles wrt homebase
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_HOMEBASE);

		// go to red bin
		NodeVec2
		MS_MOVE_TO_RED_BIN = new v_LinearAttraction_v(0.4,0.0,
			PS_RED_BIN);

		// go to home
		NodeVec2
		MS_MOVE_TO_HOMEBASE = new v_LinearAttraction_v(0.4,0.0,
			PS_HOMEBASE);

		// go to blue bin
		NodeVec2
		MS_MOVE_TO_BLUE_BIN = new v_LinearAttraction_v(0.4,0.0,
			PS_BLUE_BIN);

		// go to red target
		NodeVec2
		MS_MOVE_TO_RED = new v_LinearAttraction_v(0.4,0.0,
			PS_CLOSEST_RED);

		// go to blue target
		NodeVec2
		MS_MOVE_TO_BLUE = new v_LinearAttraction_v(0.4,0.0,
			PS_CLOSEST_BLUE);

		// swirl obstacles wrt noise
		NodeVec2
		MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS,
			MS_NOISE_VECTOR);


		//================
		// AS_WANDER
		//================
		// wander anywhere
		v_StaticWeightedSum_va 
		AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0]  = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER.weights[1]  = 1.0;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER.weights[2]  = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;

		AS_WANDER.weights[3]  = 2.0;
		AS_WANDER.embedded[3] = MS_AVOID_HOME_ZONE;


		//================
		// AS_WANDER_IN_HOME_ZONE
		//================
		// wander only in the home zone
		v_StaticWeightedSum_va 
		AS_WANDER_IN_HOME_ZONE = new v_StaticWeightedSum_va();

		AS_WANDER_IN_HOME_ZONE.weights[0]  = 1.0;
		AS_WANDER_IN_HOME_ZONE.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER_IN_HOME_ZONE.weights[1]  = 1.0;
		AS_WANDER_IN_HOME_ZONE.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER_IN_HOME_ZONE.weights[2]  = 1.0;
		AS_WANDER_IN_HOME_ZONE.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;

		AS_WANDER_IN_HOME_ZONE.weights[3]  = 1.0;
		AS_WANDER_IN_HOME_ZONE.embedded[3] = MS_AVOID_HOMEBASE;

		AS_WANDER_IN_HOME_ZONE.weights[4]  = 1.0;
		AS_WANDER_IN_HOME_ZONE.embedded[4] = 
					MS_STAY_IN_HOME_ZONE;


		//================
		// AS_WANDER_WEST
		//================
		// wander only in the west
		v_StaticWeightedSum_va 
		AS_WANDER_WEST = new v_StaticWeightedSum_va();

		AS_WANDER_WEST.weights[0]  = 1.0;
		AS_WANDER_WEST.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER_WEST.weights[1]  = 1.0;
		AS_WANDER_WEST.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER_WEST.weights[2]  = 1.0;
		AS_WANDER_WEST.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;

		AS_WANDER_WEST.weights[3]  = 1.0;
		AS_WANDER_WEST.embedded[3] = MS_AVOID_HOME_ZONE;

		AS_WANDER_WEST.weights[4]  = 1.0;
		AS_WANDER_WEST.embedded[4] = 
					MS_STAY_IN_WEST_TERRITORY;


		//================
		// AS_WANDER_EAST
		//================
		// wander only in the east
		v_StaticWeightedSum_va 
		AS_WANDER_EAST = new v_StaticWeightedSum_va();

		AS_WANDER_EAST.weights[0]  = 1.0;
		AS_WANDER_EAST.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER_EAST.weights[1]  = 1.0;
		AS_WANDER_EAST.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER_EAST.weights[2]  = 1.0;
		AS_WANDER_EAST.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;

		AS_WANDER_EAST.weights[3]  = 1.0;
		AS_WANDER_EAST.embedded[3] = MS_AVOID_HOME_ZONE;

		AS_WANDER_EAST.weights[4]  = 1.0;
		AS_WANDER_EAST.embedded[4] = 
					MS_STAY_IN_EAST_TERRITORY;


		//================
		// AS_ACQUIRE_RED
		//================
		// go to a red thing
		v_StaticWeightedSum_va 
		AS_ACQUIRE_RED = new v_StaticWeightedSum_va();

		AS_ACQUIRE_RED.weights[0]  = 1.0;
		AS_ACQUIRE_RED.embedded[0] = MS_AVOID_OBSTACLES;

		AS_ACQUIRE_RED.weights[1]  = 1.0;
		AS_ACQUIRE_RED.embedded[1] = MS_MOVE_TO_RED;

		AS_ACQUIRE_RED.weights[2]  = 1.0;
		AS_ACQUIRE_RED.embedded[2] = MS_SWIRL_OBSTACLES_RED;

		AS_ACQUIRE_RED.weights[3]  = 0.2;
		AS_ACQUIRE_RED.embedded[3] = MS_NOISE_VECTOR;

		AS_ACQUIRE_RED.weights[4]  = 1.0;
		AS_ACQUIRE_RED.embedded[4] = MS_AVOID_HOMEBASE;


		//================
		// AS_ACQUIRE_BLUE
		//================
		// go to a blue thing
		v_StaticWeightedSum_va 
		AS_ACQUIRE_BLUE = new v_StaticWeightedSum_va();

		AS_ACQUIRE_BLUE.weights[0]  = 1.0;
		AS_ACQUIRE_BLUE.embedded[0] = MS_AVOID_OBSTACLES;

		AS_ACQUIRE_BLUE.weights[1]  = 1.0;
		AS_ACQUIRE_BLUE.embedded[1] = MS_MOVE_TO_BLUE;

		AS_ACQUIRE_BLUE.weights[2]  = 1.0;
		AS_ACQUIRE_BLUE.embedded[2] = MS_SWIRL_OBSTACLES_BLUE;

		AS_ACQUIRE_BLUE.weights[3]  = 0.2;
		AS_ACQUIRE_BLUE.embedded[3] = MS_NOISE_VECTOR;

		AS_ACQUIRE_BLUE.weights[4]  = 1.0;
		AS_ACQUIRE_BLUE.embedded[4] = MS_AVOID_HOMEBASE;


		//================
		// AS_DELIVER_RED
		//================
		// head for the red bin
		v_StaticWeightedSum_va 
		AS_DELIVER_RED = new v_StaticWeightedSum_va();

		AS_DELIVER_RED.weights[0]  = 1.0;
		AS_DELIVER_RED.embedded[0] = MS_AVOID_OBSTACLES;

		AS_DELIVER_RED.weights[1]  = 1.0;
		AS_DELIVER_RED.embedded[1] = MS_MOVE_TO_RED_BIN;

		AS_DELIVER_RED.weights[2]  = 1.0;
		AS_DELIVER_RED.embedded[2] = MS_SWIRL_OBSTACLES_HOMEBASE;

		AS_DELIVER_RED.weights[3]  = 0.2;
		AS_DELIVER_RED.embedded[3] = MS_NOISE_VECTOR;

		AS_DELIVER_RED.weights[4]  = 1.0;
		AS_DELIVER_RED.embedded[4] = MS_AVOID_BLUE_BIN;

		AS_DELIVER_RED.weights[5]  = 1.0;
		AS_DELIVER_RED.embedded[5] = MS_AVOID_HOMEBASE;


		//================
		// AS_DELIVER_BLUE
		//================
		// head for the blue bin
		v_StaticWeightedSum_va 
		AS_DELIVER_BLUE = new v_StaticWeightedSum_va();

		AS_DELIVER_BLUE.weights[0]  = 1.0;
		AS_DELIVER_BLUE.embedded[0] = MS_AVOID_OBSTACLES;

		AS_DELIVER_BLUE.weights[1]  = 1.0;
		AS_DELIVER_BLUE.embedded[1] = MS_MOVE_TO_BLUE_BIN;

		AS_DELIVER_BLUE.weights[2]  = 1.0;
		AS_DELIVER_BLUE.embedded[2] = MS_SWIRL_OBSTACLES_HOMEBASE;

		AS_DELIVER_BLUE.weights[3]  = 0.2;
		AS_DELIVER_BLUE.embedded[3] = MS_NOISE_VECTOR;

		AS_DELIVER_BLUE.weights[4]  = 1.0;
		AS_DELIVER_BLUE.embedded[4] = MS_AVOID_RED_BIN;

		AS_DELIVER_BLUE.weights[5]  = 1.0;
		AS_DELIVER_BLUE.embedded[5] = MS_AVOID_HOMEBASE;


                //======
                // STATE AND ACTION
                //======
                i_FSA_ba
                STATE_MACHINE = new i_FSA_ba();
                state_monitor = STATE_MACHINE;

                STATE_MACHINE.state = 0;

                // WANDER
                //STATE_MACHINE.triggers[WANDER][0]     = PF_RED_VISIBLE;
                STATE_MACHINE.triggers[WANDER][0]       = PF_RED_VISIBLE;
                STATE_MACHINE.follow_on[WANDER][0]      = ACQUIRE_RED;
                //STATE_MACHINE.triggers[WANDER][1]       = PF_BLUE_VISIBLE;
                //STATE_MACHINE.triggers[WANDER][1]     = PF_BLUE_VISIBLE_OUT_HZ;
                //STATE_MACHINE.follow_on[WANDER][1]      = ACQUIRE_BLUE;

                // ACQUIRE_RED
                STATE_MACHINE.triggers[ACQUIRE_RED][0]  = PF_RED_IN_GRIPPER;
                STATE_MACHINE.follow_on[ACQUIRE_RED][0] = DELIVER_RED;
                STATE_MACHINE.triggers[ACQUIRE_RED][1]  = PF_NOT_RED_VISIBLE;
                STATE_MACHINE.follow_on[ACQUIRE_RED][1] = WANDER;
                STATE_MACHINE.triggers[ACQUIRE_RED][2]  = PF_BLUE_IN_GRIPPER;
                STATE_MACHINE.follow_on[ACQUIRE_RED][2] = DELIVER_BLUE;

                // DELIVER_RED
                //STATE_MACHINE.triggers[DELIVER_RED][0] = PF_CLOSE_TO_RED_BIN;
                STATE_MACHINE.triggers[DELIVER_RED][0] = PF_CLOSE_TO_RED_BIN;
                STATE_MACHINE.follow_on[DELIVER_RED][0] = WANDER;

                // ACQUIRE_BLUE
                STATE_MACHINE.triggers[ACQUIRE_BLUE][0] = PF_BLUE_IN_GRIPPER;
                STATE_MACHINE.follow_on[ACQUIRE_BLUE][0]= DELIVER_BLUE;
                STATE_MACHINE.triggers[ACQUIRE_BLUE][1] = PF_NOT_BLUE_VISIBLE;
                STATE_MACHINE.follow_on[ACQUIRE_BLUE][1]= WANDER;
                STATE_MACHINE.triggers[ACQUIRE_BLUE][2] = PF_RED_IN_GRIPPER;
                STATE_MACHINE.follow_on[ACQUIRE_BLUE][2]= DELIVER_RED;

                // DELIVER_BLUE
                STATE_MACHINE.triggers[DELIVER_BLUE][0] = PF_CLOSE_TO_BLUE_BIN;
                STATE_MACHINE.follow_on[DELIVER_BLUE][0]= WANDER;

                // DROP
                //STATE_MACHINE.triggers[DROP][0]
                //                       = PF_DROP_CRITERIA;
                //STATE_MACHINE.follow_on[DROP][0]        = WANDER;

		action_monitor = STATE_MACHINE;

                //reward
                d_ForageReward_bbbb
                REWARD = new d_ForageReward_bbbb(
                        PF_CLOSE_TO_RED_BIN,
                        PF_RED_IN_GRIPPER,
                        PF_CLOSE_TO_BLUE_BIN,
                        PF_BLUE_IN_GRIPPER);
                reward_monitor = REWARD;

		//================
		// STEERING
		//================
		v_Select_vai
		//STEERING = new v_Select_vai((NodeInt)LEARNED_ACTION);
		STEERING = new v_Select_vai((NodeInt)STATE_MACHINE);

		STEERING.embedded[0] 	= AS_WANDER;
		STEERING.embedded[1]	= AS_ACQUIRE_RED;
		STEERING.embedded[2]	= AS_ACQUIRE_BLUE;
		STEERING.embedded[3]	= AS_DELIVER_RED;
		STEERING.embedded[4]	= AS_DELIVER_BLUE;
		STEERING.embedded[5]	= AS_WANDER_IN_HOME_ZONE;
		//STEERING.embedded[2]	= AS_WANDER_WEST;
		//STEERING.embedded[3]	= AS_WANDER_EAST;
		//STEERING.embedded[8]	= MS_AVOID_HOME_ZONE;


		//================
		// TURRET
		//================
		v_Select_vai
		//TURRET = new v_Select_vai((NodeInt)LEARNED_ACTION);
                TURRET = new v_Select_vai((NodeInt)STATE_MACHINE);

		TURRET.embedded[0] 	= AS_WANDER;
		TURRET.embedded[1]	= MS_MOVE_TO_RED;
		TURRET.embedded[2]	= MS_MOVE_TO_BLUE;
		TURRET.embedded[3]	= PS_HOMEBASE; // always face home
		TURRET.embedded[4]	= PS_HOMEBASE; // always face home
		TURRET.embedded[5]	= AS_WANDER_IN_HOME_ZONE;
		//TURRET.embedded[8]	= MS_AVOID_HOME_ZONE;
		//TURRET.embedded[1]	= AS_WANDER_IN_HOME_ZONE;
		//TURRET.embedded[2]	= AS_WANDER_WEST;
		//TURRET.embedded[3]	= AS_WANDER_EAST;



		//================
		// GRIPPER_FINGERS
		//================
		d_Select_i
		//GRIPPER_FINGERS = new d_Select_i(LEARNED_ACTION);
                GRIPPER_FINGERS = new d_Select_i(STATE_MACHINE);

		GRIPPER_FINGERS.embedded[0] 	= 1;  // open 
		GRIPPER_FINGERS.embedded[1]	= -1; // trigger for acq 
		GRIPPER_FINGERS.embedded[2]	= -1; // trigger for acq
		GRIPPER_FINGERS.embedded[3]	= 0;  // closed for deliver 
		GRIPPER_FINGERS.embedded[4]	= 0;  // closed for deliver
		//GRIPPER_FINGERS.embedded[5]	= -1; // trigger for acquire
		//GRIPPER_FINGERS.embedded[6]	= 0;  // closed 
		//GRIPPER_FINGERS.embedded[7]	= 0;  // closed 
		//GRIPPER_FINGERS.embedded[8]	= 1;  // open 


		steering_configuration = STEERING;
		turret_configuration = TURRET;
		gripper_fingers_configuration = GRIPPER_FINGERS;
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

		// FINGERS
		dresult = gripper_fingers_configuration.Value(curr_time);
		abstract_robot.setGripperFingers(curr_time, dresult);

		// DISPLAY the state
		int state = state_monitor.Value(curr_time);
		int action = action_monitor.Value(curr_time);
		String msg = "blank";
		if (action == WANDER)
			msg = "AS_WANDER";
		else if (action == ACQUIRE_RED)
			msg = "AS_ACQUIRE_RED";
		else if (action == ACQUIRE_BLUE)
			msg = "AS_ACQUIRE_BLUE";
		else if (action == DELIVER_RED)
			msg = "AS_DELIVER_RED";
		else if (action == DELIVER_BLUE)
			msg = "AS_DELIVER_BLUE";
		else if (action == WANDER_IN_HOME_ZONE)
			msg = "AS_WANDER_IN_HOME_ZONE";
		else if (action == DROP)
			msg = "DROP";
		msg = msg.concat(String.valueOf(state));
		abstract_robot.setDisplayString(msg);

		// Check for reward
		double reward = reward_monitor.Value(curr_time);
		//System.out.println(abstract_robot.getPlayerNumber(curr_time));
		if (reward > 0) 
			{
			System.out.println("Reward for robot "+
				abstract_robot.getPlayerNumber(curr_time));
			num_delivered++;
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

		/*--- compose the log string ---*/
		String logmsg = num_delivered + " "
				+ 0 + " "
				+ 0 + " "
				+ 0 + "\n";
		System.out.print(logmsg);
		
		/*--- write the log string to a file ---*/
		RandomAccessFile rf;
		try
			{
			rf = new RandomAccessFile(log_filename, "rw");
			rf.seek(rf.length()); // go to end
			rf.writeBytes(logmsg);
			rf.close();
			}
		catch (Exception e)
			{
			System.out.println(e + " can't log to "+log_filename);
			}
		}
	}
