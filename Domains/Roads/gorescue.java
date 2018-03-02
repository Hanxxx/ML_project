/*
 * gorescue.java
 */

import	java.io.*;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	EDU.gatech.cc.is.learning.*;

/**
 *
 */

public class gorescue  extends ControlSystemRescueVan
	{
	private NodeVec2	steering_configuration;
	private NodeInt 	state_monitor;
	private	int	GO_TO_GOAL = 0;
	private int	NUM_GOALS = 9;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		//================
		// Set some initial hardware configurations.
		//================
		abstract_robot.setObstacleMaxRange(3.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(40);
		abstract_robot.setKinMaxRange(100);

		//================
		// perceptual schemas
		//================
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);

		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- goals
		NodeVec2[] PS_GOAL_GLOBAL = new v_FixedPoint_[NUM_GOALS];
		NodeVec2[] PS_GOAL = new v_GlobalToEgo_rv[NUM_GOALS];

		PS_GOAL_GLOBAL[0] = new v_FixedPoint_(0, 0); // USE
		PS_GOAL_GLOBAL[1] = new v_FixedPoint_(-1100.0, -550);//THM_RD_2
		PS_GOAL_GLOBAL[2] = new v_FixedPoint_(-2600,2650); //THM_RD_1
		PS_GOAL_GLOBAL[3] = new v_FixedPoint_(-4000,5000); //Person
		PS_GOAL_GLOBAL[4] = PS_GOAL_GLOBAL[2];//THM_RD_1
		PS_GOAL_GLOBAL[5] = PS_GOAL_GLOBAL[1];//THM_RD_2
		PS_GOAL_GLOBAL[6] = new v_FixedPoint_(-550, -1990);//THM_RD_3
		PS_GOAL_GLOBAL[7] = new v_FixedPoint_(-2050, -2990);//HOSP
		PS_GOAL_GLOBAL[8] = PS_GOAL_GLOBAL[6];

		for(int i=0; i<NUM_GOALS; i++)
			{
			PS_GOAL[i] = new v_GlobalToEgo_rv(abstract_robot, 
			PS_GOAL_GLOBAL[i]);
			}

		//================
		// triggers
		//================
                // close to goal
                NodeBoolean[] PF_AT_GOAL = new b_Close_vv[NUM_GOALS];
		for(int i=0; i<NUM_GOALS; i++)
			{
			PF_AT_GOAL[i] = new b_Close_vv(100, PS_GLOBAL_POS,
				PS_GOAL_GLOBAL[i]);
			}

		//================
		// motor schemas
		//================
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(9.0,
			abstract_robot.RADIUS + 0.1,
			PS_OBS);

		// go to goals
		NodeVec2[] MS_MOVE_TO_GOAL = new v_LinearAttraction_v[NUM_GOALS];
		for(int i=0; i<NUM_GOALS; i++)
			{
			MS_MOVE_TO_GOAL[i] = new v_LinearAttraction_v(0.4,0.0,
			PS_GOAL[i]);
			}

		double aso_gain = 0.0;
		double noise_gain = 0.0;
		double mtg_gain = 0.7;

		//================
		// AS_GO_TO_GOAL
		//================
		v_StaticWeightedSum_va[]
		AS_GO_TO_GOAL = new v_StaticWeightedSum_va[NUM_GOALS];
		for(int i=0; i<NUM_GOALS; i++)
			{
			AS_GO_TO_GOAL[i] = new v_StaticWeightedSum_va();

			(AS_GO_TO_GOAL[i]).weights[0]  = aso_gain;
			(AS_GO_TO_GOAL[i]).embedded[0] = MS_AVOID_OBSTACLES;
	
			(AS_GO_TO_GOAL[i]).weights[1]  = mtg_gain;
			(AS_GO_TO_GOAL[i]).embedded[1] = MS_MOVE_TO_GOAL[i];
			}

                //======
                // STATE AND ACTION
                //======
                i_FSA_ba
                STATE_MACHINE = new i_FSA_ba();
                state_monitor = STATE_MACHINE;

                STATE_MACHINE.state = 0;

		for(int i=0; i<NUM_GOALS; i++)
			{
                	STATE_MACHINE.triggers[i][0]       = PF_AT_GOAL[i];
                	STATE_MACHINE.follow_on[i][0]      = i+1;
			}

                STATE_MACHINE.follow_on[NUM_GOALS-1][0]      = 0;

                //STATE_MACHINE.triggers[3][1]       = PF_HAVE_HURT;
                //STATE_MACHINE.follow_on[3][1]      = 4;


                //================
                // STEERING
                //================
                v_Select_vai
                STEERING = new v_Select_vai((NodeInt)STATE_MACHINE);

		for(int i=0; i<NUM_GOALS; i++)
                	STEERING.embedded[i]    = AS_GO_TO_GOAL[i];

                //================
                // GRIPPER_FINGERS
                //================
                d_Select_i
                GRIPPER_FINGERS = new d_Select_i((NodeInt)STATE_MACHINE);

		for(int i=0; i<NUM_GOALS; i++)
                	GRIPPER_FINGERS.embedded[i] = 0; // closed

                GRIPPER_FINGERS.embedded[0] = 0; // trigger
                GRIPPER_FINGERS.embedded[1] = 0; // closed
                GRIPPER_FINGERS.embedded[2] = 0; // closed
                GRIPPER_FINGERS.embedded[3] = -1; // trigg
                GRIPPER_FINGERS.embedded[4] = 0; // closed
                GRIPPER_FINGERS.embedded[5] = 0; // closed
                GRIPPER_FINGERS.embedded[6] = 0; // closed
                GRIPPER_FINGERS.embedded[7] = 0; // open
                GRIPPER_FINGERS.embedded[8] = 1; // open

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

		// STEER
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, 1.0);

		// DISPLAY the state
		//int action = state_monitor.Value(curr_time);
		int action = GO_TO_GOAL;
		String msg = "BLANK";
		if (action==GO_TO_GOAL)
			msg = "GO_TO_GOAL";
				
		abstract_robot.setDisplayString(msg);

		if (done)
			return(CSSTAT_DONE);
		else
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
