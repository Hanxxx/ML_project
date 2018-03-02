/*
 * forage.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;

/**
 * <B>Introduction</B><BR>
 * Example of complex schema-based control system for a
 * MultiForageN150 robot.  
 * It uses a motor schema-based configuration to search for attractor
 * objects and carry them to a homebase.
 * The configuration is built using Clay.
 * It can be run in simulation or on a robot.
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

public class forage  extends ControlSystemMFN150
	{
	public final static boolean DEBUG = false;
	private NodeVec2	turret_configuration;
	private NodeVec2	steering_configuration;
	private NodeDouble	gripper_fingers_configuration;
	private NodeDouble	gripper_height_configuration;

	/**
	Configure the control system using Clay.
	*/
	public void Configure()
		{

		//======
		// Set some initial hardware configurations.
		//======
		if (DEBUG) System.out.println("forage .Configure()");
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
							// things further away
		abstract_robot.setBaseSpeed(0.4*abstract_robot.MAX_TRANSLATION);
		abstract_robot.setGripperHeight(-1,0);   // put gripper down
		abstract_robot.setGripperFingers(-1,-1); // put in trigger mode

		//======
		// perceptual schemas
		//======
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_SONAR_OBS = new va_Obstacles_r(abstract_robot);
		NodeVec2Array // make them global
		PS_OBS_GLOBAL = new va_Add_vav(PS_SONAR_OBS,PS_GLOBAL_POS);
		NodeVec2Array // make them persist for 10 seconds
		PS_OBS_GLOBAL_PERS = new va_PersistBlend_va(
			10,0.05,0.75,PS_OBS_GLOBAL);
		NodeVec2Array // make then egocentric
		PS_OBS = new va_Subtract_vav(PS_OBS_GLOBAL_PERS,PS_GLOBAL_POS);
		NodeVec2      // find the closest one
		PS_CLOSEST_OBS = new v_Closest_va(PS_OBS);

		//--- homebase 
		NodeVec2      // a fixed position in front of the door
		PS_HOMEBASE0_GLOBAL = new v_FixedPoint_(0.0,-1.63);
		NodeVec2      // make it egocentric
		PS_HOMEBASE0 = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE0_GLOBAL);

		//--- blind door 3
		// in case we don't see the door, go to this spot
		NodeVec2      // the fixed spot
		PS_BLIND_DOOR3_GLOBAL = new v_FixedPoint_(0.0,-.38);
		NodeVec2      // make it egocentric
		PS_BLIND_DOOR3 = new v_GlobalToEgo_rv(abstract_robot,
				PS_BLIND_DOOR3_GLOBAL);

		//--- homebase for type 1 targets
		NodeVec2
		PS_HOMEBASE1_GLOBAL = new v_FixedPoint_(0.0,1.63);
		NodeVec2
		PS_HOMEBASE1 = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE1_GLOBAL);

		//--- blind door 4
		// in case we don't see the door, go to this spot
		NodeVec2
		PS_BLIND_DOOR4_GLOBAL = new v_FixedPoint_(0.0,.38);
		NodeVec2
		PS_BLIND_DOOR4 = new v_GlobalToEgo_rv(abstract_robot,
				PS_BLIND_DOOR4_GLOBAL);

		//--- targets in channel 0
		NodeVec2Array 
		PS_TARGETS0 = 
			new va_VisualObjects_r(0,abstract_robot);

		//--- targets in channel 1
		NodeVec2Array 
		PS_TARGETS1 = 
			new va_VisualObjects_r(1,abstract_robot);
		
		//--- targets in channel 2
		NodeVec2Array 
		PS_TARGETS2 = 
			new va_VisualObjects_r(2,abstract_robot);
		
		//--- all targets
		NodeVec2Array 
		PS_TARGETS01 = new va_Merge_vava(PS_TARGETS0, PS_TARGETS1);
		NodeVec2Array 
		PS_TARGETS = new va_Merge_vava(PS_TARGETS01, PS_TARGETS2);
		NodeVec2Array 
		PS_TARGETS_GLOBAL = new va_Add_vav(PS_TARGETS, 
					PS_GLOBAL_POS);
		NodeVec2Array 
		PS_TARGETS_GLOBAL_FILT1 = new va_FilterOutClose_vva(0.75,
			PS_BLIND_DOOR3_GLOBAL,PS_TARGETS_GLOBAL);
		NodeVec2Array 
		PS_TARGETS_GLOBAL_FILT2 = new va_FilterOutClose_vva(0.75,
			PS_BLIND_DOOR4_GLOBAL,PS_TARGETS_GLOBAL_FILT1);
		NodeVec2Array 
		PS_TARGETS_GLOBAL_PERS = new va_PersistBlend_va(2,0.3,0.75,
					PS_TARGETS_GLOBAL_FILT2);
		NodeVec2Array 
		PS_TARGETS_EGO_PERS = new va_Subtract_vav(PS_TARGETS_GLOBAL_PERS,
					PS_GLOBAL_POS);

		//--- the closest target
		NodeVec2
		PS_CLOSEST = new v_Closest_va(PS_TARGETS_EGO_PERS);

		//--- doors in channel 3
		NodeVec2Array 
		PS_DOORS3_RAW = 
			new va_VisualObjects_r(3,abstract_robot);
		NodeVec2Array 
		PS_DOORS3_GLOBAL = new va_Add_vav(PS_DOORS3_RAW, 
					PS_GLOBAL_POS);
		NodeVec2Array 
		PS_DOORS3_GLOBAL_PERS = new va_PersistBlend_va(60,1.0,0.75,
					PS_DOORS3_GLOBAL);
		NodeVec2Array 
		PS_DOORS3_EGO_PERS = new va_Subtract_vav(PS_DOORS3_GLOBAL_PERS,
					PS_GLOBAL_POS);

		//--- the closest 3 door
		NodeVec2
		PS_CLOSEST_DOOR3 = new v_Closest_va(PS_DOORS3_EGO_PERS);

		//--- doors in channel 4
		NodeVec2Array 
		PS_DOORS4_RAW = 
			new va_VisualObjects_r(4,abstract_robot);
		NodeVec2Array 
		PS_DOORS4_GLOBAL = new va_Add_vav(PS_DOORS4_RAW, 
					PS_GLOBAL_POS);
		NodeVec2Array 
		PS_DOORS4_GLOBAL_PERS = new va_PersistBlend_va(60,1.0,0.75,
					PS_DOORS4_GLOBAL);
		NodeVec2Array 
		PS_DOORS4_EGO_PERS = new va_Subtract_vav(PS_DOORS4_GLOBAL_PERS,
					PS_GLOBAL_POS);
		//--- the closest4  door
		NodeVec2
		PS_CLOSEST_DOOR4 = new v_Closest_va(PS_DOORS4_EGO_PERS);

		//--- type of object in the gripper
		NodeInt
		PS_IN_GRIPPER = new i_InGripper_r(abstract_robot);


		//======
		// Perceptual Features
		//======
		// is something visible?
		NodeBoolean
		PF_SOMETHING_VISIBLE = new b_NonZero_v(PS_CLOSEST);
		NodeBoolean
		PF_NOT_SOMETHING_VISIBLE = new b_Not_s(PF_SOMETHING_VISIBLE);

		//is a Door3 visible?
		NodeBoolean
		PF_DOOR3_VISIBLE = new b_NonZero_v(PS_CLOSEST_DOOR3);
		NodeBoolean
		PF_NOT_DOOR3_VISIBLE = new b_Not_s(PF_DOOR3_VISIBLE);

		//is a Door4 visible?
		NodeBoolean
		PF_DOOR4_VISIBLE = new b_NonZero_v(PS_CLOSEST_DOOR4);
		NodeBoolean
		PF_NOT_DOOR4_VISIBLE = new b_Not_s(PF_DOOR4_VISIBLE);

		// is something in the gripper?
		NodeBoolean  // remember it for 2 seconds
		PF_SOMETHING_IN_GRIPPER = new b_Persist_s(4.0,
				new b_NonNegative_s(PS_IN_GRIPPER));
		NodeBoolean
		PF_NOT_SOMETHING_IN_GRIPPER = new b_Not_s(PF_SOMETHING_IN_GRIPPER);

		// progress?
		NodeBoolean  
		PF_WATCHDOG = new b_WatchDog_s(60.0, 20, PF_SOMETHING_IN_GRIPPER);
		NodeBoolean
		PF_NOT_WATCHDOG = new b_Not_s(PF_WATCHDOG);

		NodeBoolean
		PF_TARGET0_IN_GRIPPER = new b_Equal_i(0,PS_IN_GRIPPER);

		// close to homebase (e.g. begin docking procedure)
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE0 = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_HOMEBASE0_GLOBAL);

		// close to homebase (e.g. begin docking procedure)
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE1 = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_HOMEBASE1_GLOBAL);

		// are we close enough to be docked?
		NodeBoolean  // if closest obstacle is within 0.42 m
		PF_DOCKED_RAW = new b_Close_vv(0.42,
				PS_CLOSEST_OBS,new v_FixedPoint_(0,0));
		NodeBoolean  // remember for 5 seconds
		PF_DOCKED = new b_Persist_s(3.0,PF_DOCKED_RAW);
		NodeBoolean
		PF_NOT_DOCKED = new b_Not_s(PF_DOCKED);


		//======
		// motor schemas
		//======
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(1.5,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// swirl obstacles wrt target
		NodeVec2
		MS_SWIRL_OBSTACLES_TARGET = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST);

		// swirl obstacles wrt door 3
		NodeVec2
		MS_SWIRL_OBSTACLES_DOOR3 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST_DOOR3);

		// swirl obstacles wrt door 4
		NodeVec2
		MS_SWIRL_OBSTACLES_DOOR4 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_CLOSEST_DOOR4);

		// swirl obstacles wrt homebase0
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE0 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_HOMEBASE0);

		// swirl obstacles wrt homebase1
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE1 = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.22,
			PS_OBS,
			PS_HOMEBASE1);

		// intercept the target
		NodeVec2
		MS_MOVE_TO_TARGET = new v_LinearAttraction_v(0.0,0.0,PS_CLOSEST);

		// go home 0
		NodeVec2
		MS_MOVE_TO_HOMEBASE0 = new v_LinearAttraction_v(0.4,0.0,PS_HOMEBASE0);

		// go home 1
		NodeVec2
		MS_MOVE_TO_HOMEBASE1 = new v_LinearAttraction_v(0.4,0.0,PS_HOMEBASE1);

		// go to door3
		NodeVec2
		MS_MOVE_TO_DOOR3 = new v_LinearAttraction_v(0.0,0.0,PS_CLOSEST_DOOR3);

		// blind go to door3
		NodeVec2
		MS_BLIND_MOVE_TO_DOOR3 = new v_LinearAttraction_v(0.0,0.0,PS_BLIND_DOOR3);

		// go to door4
		NodeVec2
		MS_MOVE_TO_DOOR4 = new v_LinearAttraction_v(0.0,0.0,PS_CLOSEST_DOOR4);

		// blind go to door4
		NodeVec2
		MS_BLIND_MOVE_TO_DOOR4 = new v_LinearAttraction_v(0.0,0.0,PS_BLIND_DOOR4);

		// search vector
		NodeVec2
		MS_SEARCH_VECTOR = new v_Noise_(30);

		// swirl obstacles
		NodeVec2
		MS_SWIRL_OBSTACLES_SEARCH = new v_Swirl_vav(2.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS,
			MS_SEARCH_VECTOR);


		//======
		// AS_SEARCH
		//======
		v_StaticWeightedSum_va 
		AS_SEARCH = new v_StaticWeightedSum_va();

		AS_SEARCH.weights[0]  = 2.0;
		AS_SEARCH.embedded[0] = MS_AVOID_OBSTACLES;

		AS_SEARCH.weights[1]  = 1.0;
		AS_SEARCH.embedded[1] = MS_SEARCH_VECTOR;

		AS_SEARCH.weights[2]  = 1.0;
		AS_SEARCH.embedded[2] = MS_SWIRL_OBSTACLES_SEARCH;



		//======
		// AS_GO_TO_TARGET
		//======
		v_StaticWeightedSum_va 
		AS_GO_TO_TARGET = new v_StaticWeightedSum_va();

		AS_GO_TO_TARGET.weights[0]  = 2.0;
		AS_GO_TO_TARGET.embedded[0] = MS_AVOID_OBSTACLES;

		AS_GO_TO_TARGET.weights[1]  = 1.0;
		AS_GO_TO_TARGET.embedded[1] = MS_MOVE_TO_TARGET;

		AS_GO_TO_TARGET.weights[2]  = 1.0;
		AS_GO_TO_TARGET.embedded[2] = MS_SWIRL_OBSTACLES_TARGET;


		//======
		// AS_DOCK3
		//======
		v_StaticWeightedSum_va 
		AS_DOCK3 = new v_StaticWeightedSum_va();

		AS_DOCK3.weights[0]  = 0.9;
		AS_DOCK3.embedded[0] = MS_MOVE_TO_DOOR3;


		//======
		// AS_BLIND_DOCK3
		//======
		v_StaticWeightedSum_va 
		AS_BLIND_DOCK3 = new v_StaticWeightedSum_va();

		AS_BLIND_DOCK3.weights[0]  = 0.9;
		AS_BLIND_DOCK3.embedded[0] = MS_BLIND_MOVE_TO_DOOR3;


		//======
		// AS_DOCK4
		//======
		v_StaticWeightedSum_va 
		AS_DOCK4 = new v_StaticWeightedSum_va();

		AS_DOCK4.weights[0]  = 0.9;
		AS_DOCK4.embedded[0] = MS_MOVE_TO_DOOR4;

		//AS_DOCK.weights[0]  = 0.1;
		//AS_DOCK.embedded[0] = MS_SWIRL_OBSTACLES_DOOR3;


		//======
		// AS_BLIND_DOCK4
		//======
		v_StaticWeightedSum_va 
		AS_BLIND_DOCK4 = new v_StaticWeightedSum_va();

		AS_BLIND_DOCK4.weights[0]  = 0.9;
		AS_BLIND_DOCK4.embedded[0] = MS_BLIND_MOVE_TO_DOOR4;


		//======
		// AS_BACKUP
		//======
		v_StaticWeightedSum_va 
		AS_BACKUP = new v_StaticWeightedSum_va();

		AS_BACKUP.weights[0]  = 0.6;
		AS_BACKUP.embedded[0] = MS_AVOID_OBSTACLES;


		//======
		// AS_GO_TO_HOMEBASE0
		//======
		v_StaticWeightedSum_va 
		AS_GO_TO_HOMEBASE0 = new v_StaticWeightedSum_va();

		AS_GO_TO_HOMEBASE0.weights[0]  = 2.0;
		AS_GO_TO_HOMEBASE0.embedded[0] = MS_AVOID_OBSTACLES;

		AS_GO_TO_HOMEBASE0.weights[1]  = 1.0;
		AS_GO_TO_HOMEBASE0.embedded[1] = MS_MOVE_TO_HOMEBASE0;

		AS_GO_TO_HOMEBASE0.weights[2]  = 1.0;
		AS_GO_TO_HOMEBASE0.embedded[2] = MS_SWIRL_OBSTACLES_HOMEBASE0;



		//======
		// AS_GO_TO_HOMEBASE1
		//======
		v_StaticWeightedSum_va 
		AS_GO_TO_HOMEBASE1 = new v_StaticWeightedSum_va();

		AS_GO_TO_HOMEBASE1.weights[0]  = 2.0;
		AS_GO_TO_HOMEBASE1.embedded[0] = MS_AVOID_OBSTACLES;

		AS_GO_TO_HOMEBASE1.weights[1]  = 1.0;
		AS_GO_TO_HOMEBASE1.embedded[1] = MS_MOVE_TO_HOMEBASE1;

		AS_GO_TO_HOMEBASE1.weights[2]  = 1.0;
		AS_GO_TO_HOMEBASE1.embedded[2] = MS_SWIRL_OBSTACLES_HOMEBASE1;


		//======
		// STATE_MACHINE
		//======
		i_FSA_ba 
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;
		
		// STATE 0 SEARCH
		STATE_MACHINE.triggers[0][0]  = PF_SOMETHING_VISIBLE;
		STATE_MACHINE.follow_on[0][0] = 1; // transition to ACQUIRE
		STATE_MACHINE.triggers[0][1]  = PF_WATCHDOG;
		STATE_MACHINE.follow_on[0][1] = 19; // transition to NONPROG

		// STATE 1 ACQUIRE
		STATE_MACHINE.triggers[1][0]  = PF_SOMETHING_IN_GRIPPER;
		STATE_MACHINE.follow_on[1][0] = 12; // transition to DELIVER
		STATE_MACHINE.triggers[1][1]  = PF_NOT_SOMETHING_VISIBLE;
		STATE_MACHINE.follow_on[1][1] = 0; // transition to SEARCH
		STATE_MACHINE.triggers[1][2]  = PF_WATCHDOG;
		STATE_MACHINE.follow_on[1][2] = 19; // transition to NONPROG

		// STATE 2 DELIVER0
		STATE_MACHINE.triggers[2][0]  = PF_NOT_SOMETHING_IN_GRIPPER;
		STATE_MACHINE.follow_on[2][0] = 0; // transition to SEARCH
		STATE_MACHINE.triggers[2][1]  = PF_CLOSE_TO_HOMEBASE0;
		STATE_MACHINE.follow_on[2][1] = 3; // transition to DOCK

		// STATE 3 DOCK3
		STATE_MACHINE.triggers[3][0]  = PF_DOCKED;
		STATE_MACHINE.follow_on[3][0] = 4; // BACKUP1
		STATE_MACHINE.triggers[3][1]  = PF_NOT_DOOR3_VISIBLE;
		STATE_MACHINE.follow_on[3][1] = 6; // BLIND dock

		// STATE 4 BACKUP1
		STATE_MACHINE.triggers[4][0]  = PF_NOT_DOCKED;
		STATE_MACHINE.follow_on[4][0] = 0; // continue SEARCH
		STATE_MACHINE.triggers[4][1]  = PF_DOCKED;
		STATE_MACHINE.follow_on[4][1] = 5; // continue to BACKUP2

		// STATE 5 BACKUP2
		STATE_MACHINE.triggers[5][0]  = PF_NOT_DOCKED;
		STATE_MACHINE.follow_on[5][0] = 0; // continue SEARCH

		// STATE 6 BLINDDOCK3
		STATE_MACHINE.triggers[6][0]  = PF_DOCKED;
		STATE_MACHINE.follow_on[6][0] = 4; // BACKUP
		STATE_MACHINE.triggers[6][1]  = PF_DOOR3_VISIBLE;
		STATE_MACHINE.follow_on[6][1] = 3; // VIS dock

		// STATE 12 DELIVER1
		STATE_MACHINE.triggers[12][0]  = PF_TARGET0_IN_GRIPPER;
		STATE_MACHINE.follow_on[12][0] = 2; // DELIVER0
		STATE_MACHINE.triggers[12][1]  = PF_NOT_SOMETHING_IN_GRIPPER;
		STATE_MACHINE.follow_on[12][1] = 0; // transition to SEARCH
		STATE_MACHINE.triggers[12][2]  = PF_CLOSE_TO_HOMEBASE1;
		STATE_MACHINE.follow_on[12][2] = 13; // transition to DOCK3

		// STATE 13 DOCK4
		STATE_MACHINE.triggers[13][0]  = PF_DOCKED;
		STATE_MACHINE.follow_on[13][0] = 14; // BACKUP
		STATE_MACHINE.triggers[13][1]  = PF_NOT_DOOR3_VISIBLE;
		STATE_MACHINE.follow_on[13][1] = 16; // BLIND dock

		// STATE 14 BACKUP1
		STATE_MACHINE.triggers[14][0]  = PF_NOT_DOCKED;
		STATE_MACHINE.follow_on[14][0] = 0; // continue SEARCH
		STATE_MACHINE.triggers[14][1]  = PF_DOCKED;
		STATE_MACHINE.follow_on[14][1] = 15; // continue to BACKUP2

		// STATE 15 BACKUP2
		STATE_MACHINE.triggers[15][0]  = PF_NOT_DOCKED;
		STATE_MACHINE.follow_on[15][0] = 0; // continue SEARCH

		// STATE 16 BLINDDOCK4
		STATE_MACHINE.triggers[16][0]  = PF_DOCKED;
		STATE_MACHINE.follow_on[16][0] = 14; // BACKUP
		STATE_MACHINE.triggers[16][1]  = PF_DOOR3_VISIBLE;
		STATE_MACHINE.follow_on[16][1] = 13; // VIS dock

		// STATE 19 NONPROGRESS
		STATE_MACHINE.triggers[19][0]  = PF_NOT_WATCHDOG;
		STATE_MACHINE.follow_on[19][0] = 0; // SEARCH
		STATE_MACHINE.triggers[19][1]  = PF_SOMETHING_IN_GRIPPER;
		STATE_MACHINE.follow_on[19][1] = 12; // transition to DELIVER1


		//======
		// STEERING
		//======
		v_Select_vai
		STEERING = new v_Select_vai((NodeInt)STATE_MACHINE);

		STEERING.embedded[0] = AS_SEARCH;
		STEERING.embedded[1] = AS_GO_TO_TARGET;
		STEERING.embedded[2] = AS_GO_TO_HOMEBASE0;
		STEERING.embedded[3] = AS_DOCK3;
		STEERING.embedded[4] = AS_BACKUP;
		STEERING.embedded[5] = AS_BACKUP;
		STEERING.embedded[6] = AS_BLIND_DOCK3;
		STEERING.embedded[12]= AS_GO_TO_HOMEBASE1;
		STEERING.embedded[13]= AS_DOCK4;
		STEERING.embedded[14]= AS_BACKUP;
		STEERING.embedded[15]= AS_BACKUP;
		STEERING.embedded[16] = AS_BLIND_DOCK4;
		STEERING.embedded[19]= AS_SEARCH;


		//======
		// TURRET
		//======
		v_Select_vai
		TURRET = new v_Select_vai((NodeInt)STATE_MACHINE);

		TURRET.embedded[0] = AS_SEARCH;
		TURRET.embedded[1] = PS_CLOSEST;
		TURRET.embedded[2] = AS_BLIND_DOCK3;
		TURRET.embedded[3] = AS_DOCK3;
		TURRET.embedded[4] = AS_DOCK3;
		TURRET.embedded[5] = AS_DOCK3;
		TURRET.embedded[6] = AS_BLIND_DOCK3;
		TURRET.embedded[12]= AS_BLIND_DOCK4;
		TURRET.embedded[13]= AS_DOCK4;
		TURRET.embedded[14]= AS_DOCK4;
		TURRET.embedded[15]= AS_DOCK4;
		TURRET.embedded[16]= AS_BLIND_DOCK4;
		TURRET.embedded[19]= AS_SEARCH;


		//======
		// GRIPPER_FINGERS
		//======
		d_Select_i
		GRIPPER_FINGERS = new d_Select_i(STATE_MACHINE);

		GRIPPER_FINGERS.embedded[0] = -1; // trigger in SEARCH
		GRIPPER_FINGERS.embedded[1] = -1; // trigger in ACQUIRE
		GRIPPER_FINGERS.embedded[2] = 0;  // closed in DELIVER
		GRIPPER_FINGERS.embedded[3] = 0;  // closed in DOCK
		GRIPPER_FINGERS.embedded[4] = 1;  // open in BACKUP1
		GRIPPER_FINGERS.embedded[5] = 0;  // closed in BACKUP2
		GRIPPER_FINGERS.embedded[6] = 0;  // closed in DOCK
		GRIPPER_FINGERS.embedded[12]= 0;  // closed in DELIVER
		GRIPPER_FINGERS.embedded[13]= 0;  // closed in DOCK
		GRIPPER_FINGERS.embedded[14]= 1;  // open in BACKUP1
		GRIPPER_FINGERS.embedded[15]= 0;  // closed in BACKUP2
		GRIPPER_FINGERS.embedded[16]= 0;  // closed in DOCK
		GRIPPER_FINGERS.embedded[19]= -1; // trigger in NONPROGRESS


		//======
		// GRIPPER_HEIGHT
		//======
		d_Select_i
		GRIPPER_HEIGHT = new d_Select_i(STATE_MACHINE);

		GRIPPER_HEIGHT.embedded[0] = 0;  // down in SEARCH
		GRIPPER_HEIGHT.embedded[1] = 0;  // down in ACQUIRE
		GRIPPER_HEIGHT.embedded[2] = 1;  // up in DELIVER
		GRIPPER_HEIGHT.embedded[3] = 1;  // up in DOCK
		GRIPPER_HEIGHT.embedded[4] = 1;  // up in BACKUP1
		GRIPPER_HEIGHT.embedded[5] = 1;  // up in BACKUP2
		GRIPPER_HEIGHT.embedded[6] = 1;  // up in DOCK
		GRIPPER_HEIGHT.embedded[12]= 1;  // up in DELIVER
		GRIPPER_HEIGHT.embedded[13]= 1;  // up in DOCK
		GRIPPER_HEIGHT.embedded[14]= 1;  // up in BACKUP1
		GRIPPER_HEIGHT.embedded[15]= 1;  // up in BACKUP2
		GRIPPER_HEIGHT.embedded[16]= 1;  // up in BACKUP2
		GRIPPER_HEIGHT.embedded[19]= 0;  // down in NONPROGRESS


		turret_configuration = TURRET;
		steering_configuration = STEERING;
		gripper_fingers_configuration = GRIPPER_FINGERS;
		gripper_height_configuration = GRIPPER_HEIGHT;
		}
		
	/**
	Called every timestep to allow the control system to
	run.
	*/
	public int TakeStep()
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

		// HEIGHT
		dresult = gripper_height_configuration.Value(curr_time);
		abstract_robot.setGripperHeight(curr_time, dresult);
	
		return(CSSTAT_OK);
		}
	
	}
