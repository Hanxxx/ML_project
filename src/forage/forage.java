package forage;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

import TBSim.SimulationCanvas;
import EDU.gatech.cc.is.simulation.AttractorSim;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;

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
 * @version $Revision: 1.2 $
 */
  
public class forage  extends ControlSystemMFN150
	{

	public final static boolean DEBUG = true;
	private NodeVec2	turret_configuration;
	private NodeVec2	steering_configuration;
	private NodeDouble	gripper_fingers_configuration;
	private NodeDouble	gripper_height_configuration;
	private NodeBoolean PF_TARGET0_VISIBLE_takestep;
	private NodeBoolean PF_TARGET0_IN_GRIPPER_takestep;
	private NodeBoolean PF_CLOSE_TO_HOMEBASE0_taekstep;
	private NodeVec2    PS_GLOBAL_POS_taekstep;
	private NodeInt 	state_monitor;
	private i_FSA_ba ACTION;
	private NodeInt STATE;
	private int state_mechine = 0;
	private int count;
	public static int count_cap;
	//=======================================================================
	private double  q[][];                  // the q-values
	private double  gamma=0.8;              // discount rate
	private double  alpha=0.2;              // learning rate
	private Random  rgen;                   // the random number generator
	private int     ls = 0;                     // last state
	private int     la = 0;                     // last action
	private double  exploration_rate = 0.2; // 
	private long	seed=0;			// random number seed
	private int     remain = 0;
	private double     rn ;                     //reward
	private double     rn_now;  
    private double  Vmax [];  //very bad
    private int max_q_action = 0;
    private int prefered_action[];

	
	
	//MultiForageN150Sim abstract_robot = new MultiForageN150Sim();
	//public static void main(String []args){
		//forage f= new forage();
		//f.configure();
		//f.takeStep();
    //}//定义主类

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		count = 1;
		rn = -1;
		count_cap = 1;
		//======
		// Set some initial hardware configurations.
		//======
		abstract_robot.setObstacleMaxRange(1.0); // don't consider 
	 						 // things further away
		abstract_robot.setBaseSpeed(0.4*abstract_robot.MAX_TRANSLATION);
		abstract_robot.setGripperHeight(-1,0);   // put gripper down
		abstract_robot.setGripperFingers(-1,1);  // open gripper
   
		//======
		// perceptual schemas
		//======
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);
		PS_GLOBAL_POS_taekstep = PS_GLOBAL_POS;
		
		//--- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot);

		//--- homebase 
		NodeVec2      // the place to deliver
		PS_HOMEBASE0_GLOBAL = new v_FixedPoint_(0.0,0.0);
		NodeVec2      // make it egocentric
		PS_HOMEBASE0 = new v_GlobalToEgo_rv(abstract_robot,
				PS_HOMEBASE0_GLOBAL);

		//--- targets of visual class 0
		NodeVec2Array 
		PS_TARGETS0_EGO = 
			new va_VisualObjects_r(0,abstract_robot);
		NodeVec2Array 
		PS_TARGETS0_GLOBAL = 
			new va_Add_vav(PS_TARGETS0_EGO, PS_GLOBAL_POS);

		//--- filter out targets close to homebase
		NodeVec2Array 
		PS_TARGETS0_GLOBAL_FILT = new va_FilterOutClose_vva(0.5,
			PS_HOMEBASE0_GLOBAL, PS_TARGETS0_GLOBAL);

		//--- make them egocentric
		NodeVec2Array 
		PS_TARGETS0_EGO_FILT = new va_Subtract_vav(
			PS_TARGETS0_GLOBAL_FILT, PS_GLOBAL_POS);

		//--- get the closest one
		NodeVec2
		PS_CLOSEST0 = new v_Closest_va(PS_TARGETS0_EGO_FILT);

		//--- type of object in the gripper
		NodeInt
		PS_IN_GRIPPER = new i_InGripper_r(abstract_robot);


		//======
		// Perceptual Features
		//======
		// is something visible?
		NodeBoolean
		PF_TARGET0_VISIBLE = new b_NonZero_v(PS_CLOSEST0);
		PF_TARGET0_VISIBLE_takestep = PF_TARGET0_VISIBLE;

		// is it not visible?
		NodeBoolean
		PF_NOT_TARGET0_VISIBLE = new b_Not_s(PF_TARGET0_VISIBLE);

		// is something in the gripper?
		NodeBoolean
		PF_TARGET0_IN_GRIPPER = new b_Equal_i(0,PS_IN_GRIPPER);
		PF_TARGET0_IN_GRIPPER_takestep = PF_TARGET0_IN_GRIPPER;

		// close to homebase 
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE0 = new b_Close_vv(0.5, PS_GLOBAL_POS,
			PS_HOMEBASE0_GLOBAL);
		PF_CLOSE_TO_HOMEBASE0_taekstep = PF_CLOSE_TO_HOMEBASE0;

		//======
		// motor schemas
		//======
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va(0.5,
			abstract_robot.RADIUS + 0.01, 
			PS_OBS);//PS_OBS--- obstacles

		// swirl obstacles wrt target 0
		NodeVec2
		MS_SWIRL_OBSTACLES_TARGET0 = new v_Swirl_vav(0.5,
			abstract_robot.RADIUS + 0.01,
			PS_OBS,
			PS_CLOSEST0);

		// swirl obstacles wrt homebase0
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE0 = new v_Swirl_vav(0.5,
			abstract_robot.RADIUS + 0.01,
			PS_OBS,
			PS_HOMEBASE0);

		// go home 0
		NodeVec2
		MS_MOVE_TO_HOMEBASE0 = new v_LinearAttraction_v(1.0,0.0,PS_HOMEBASE0);

		// go to target0
		NodeVec2
		MS_MOVE_TO_TARGET0 = new v_LinearAttraction_v(1.0,0.0,PS_CLOSEST0);

		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(5,seed);

		// swirl obstacles wrt noise
		NodeVec2
		MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(0.5,
			abstract_robot.RADIUS + 0.01,
			PS_OBS,
			MS_NOISE_VECTOR);


		//======
		// AS_WANDER
		//======
		v_StaticWeightedSum_va 
		AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0]  = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES;

		AS_WANDER.weights[1]  = 1.0;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR;

		AS_WANDER.weights[2]  = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE;


		//======
		// AS_GO_TO_TARGET0
		//======
		v_StaticWeightedSum_va 
		AS_ACQUIRE0 = new v_StaticWeightedSum_va();

		AS_ACQUIRE0.weights[0]  = 1.0;
		AS_ACQUIRE0.embedded[0] = MS_AVOID_OBSTACLES;

		AS_ACQUIRE0.weights[1]  = 1.0;
		AS_ACQUIRE0.embedded[1] = MS_MOVE_TO_TARGET0;

		AS_ACQUIRE0.weights[2]  = 1.0;
		AS_ACQUIRE0.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0;

		AS_ACQUIRE0.weights[3]  = 0.1;
		AS_ACQUIRE0.embedded[3] = MS_NOISE_VECTOR;



		//======
		// AS_DELIVER0
		//======
		v_StaticWeightedSum_va 
		AS_DELIVER0 = new v_StaticWeightedSum_va();

		AS_DELIVER0.weights[0]  = 1.0;
		AS_DELIVER0.embedded[0] = MS_AVOID_OBSTACLES;

		AS_DELIVER0.weights[1]  = 1.0;
		AS_DELIVER0.embedded[1] = MS_MOVE_TO_HOMEBASE0;

		AS_DELIVER0.weights[2]  = 1.0;
		AS_DELIVER0.embedded[2] = MS_SWIRL_OBSTACLES_HOMEBASE0;

		AS_DELIVER0.weights[3]  = 0.1;
		AS_DELIVER0.embedded[3] = MS_NOISE_VECTOR;

		
	
		
		//======
		// STATE_MACHINE
		//======
		ACTION = new i_FSA_ba();

		ACTION.state = 0;
		
		// STATE 0 WANDER
		//ACTION.triggers[0][0]  = PF_TARGET0_VISIBLE;// is something visible?
		//ACTION.follow_on[0][0] = 1; // transition to ACQUIRE0

		// STATE 1 ACQUIRE0
		//ACTION.triggers[1][0]  = PF_TARGET0_IN_GRIPPER;// is something in the gripper?
		//ACTION.follow_on[1][0] = SimulationCanvas.zero; // transition to DELIVER
		//ACTION.triggers[1][1]  = PF_NOT_TARGET0_VISIBLE;// is it not visible?
		//ACTION.follow_on[1][1] = 0; // transition to WANDER

		// STATE 2 DELIVER0
		//ACTION.triggers[2][0]  = PF_CLOSE_TO_HOMEBASE0;// close to homebase 
		//ACTION.follow_on[2][0] = 0; // transition to WANDER

		state_monitor = ACTION;


		//======
		// STEERING
		//======
		v_Select_vai
		STEERING = new v_Select_vai((NodeInt)ACTION);

		STEERING.embedded[0] = AS_WANDER;
		STEERING.embedded[1] = AS_ACQUIRE0;
		STEERING.embedded[2] = AS_DELIVER0;


		//======
		// TURRET
		//======
		v_Select_vai
		TURRET = new v_Select_vai((NodeInt)ACTION);

		TURRET.embedded[0] = AS_WANDER;
		TURRET.embedded[1] = AS_ACQUIRE0;
		TURRET.embedded[2] = AS_DELIVER0;


		//======
		// GRIPPER_FINGERS
		//======
		d_Select_i
		GRIPPER_FINGERS = new d_Select_i(ACTION);

		GRIPPER_FINGERS.embedded[0] = 1;  // open in WANDER
		GRIPPER_FINGERS.embedded[1] = -1; // trigger in ACQUIRE
		GRIPPER_FINGERS.embedded[2] = 0;  // closed in DELIVER


		turret_configuration = TURRET;
		steering_configuration = STEERING;
		gripper_fingers_configuration = GRIPPER_FINGERS;
		
		/**
		//初始化Q值表
		rgen = new Random(seed);
		q = new double[8][3];
        for(int i=0; i<8; i++)
        {
        	for(int j=0; j<3; j++)
        	{
        		q[i][j] = rgen.nextDouble()*2 - 1;
        	}
        }
        */
		//从文件中读入数据
		
		double[][] arr2 = new double[8][3];
		q = new double[8][3];
		File file = new File("d:\\q.txt"); 
		  BufferedReader in = null;
		try {
			in = new BufferedReader(new FileReader(file));
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} 
		  String line;  //一行数据
		  int row=0;
		  //逐行读取，并将每个数组放入到数组中
		  try {
			while((line = in.readLine()) != null)
			  {
				  String[] temp = line.split("\t");  
				  for(int j=0;j<temp.length;j++)
				  {
					  arr2[row][j] = Double.parseDouble(temp[j]);
				   }
			   row++;
			   }
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		  try {
			in.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		  
		  q = arr2;
		
        //初始化状态选择
        prefered_action = new int [8];
        Vmax = new double [8];
        for (int t = 0; t < 8; t++)
        {
        	Vmax [t] = -9999999999f;
        }
        int rd_q_action;
        for(int j = 0; j < 8; j++)
        {
        for (int i = 0; i < 3; i++)
                {
                if (q[j][i] > Vmax [j])
                        {
                        Vmax [j] = q[j][i];
                        max_q_action = i;
                        }
                }
        double rd=Math.random();
        Random random = new Random();
        rd_q_action = random.nextInt(3)%(3) + 1;//random.nextInt(max)%(max-min+1) + min;
        if (rd<exploration_rate)
        {
        	prefered_action[j] = rd_q_action - 1;
        }
        else
        {
        	prefered_action[j] = max_q_action;
        }
        }
        
        ls = 0;
        la = prefered_action[0];
        
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
		
		//============================================
		//选择要执行的action
		//============================================



        	
		//============================================
		// STATE定义状态
		//============================================
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==0)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==0)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==0))
			{
			state_mechine = 0;
			System.out.println("state_0");
			ACTION.state = 0;//prefered_action[0];
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==0)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==0)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==1))
		{
			state_mechine = 1;
			System.out.println("state_1");
			ACTION.state = 1;//prefered_action[1];
		}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==0)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==1)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==0))
			{
			state_mechine = 2;
			System.out.println("state_2");
			ACTION.state = 2;//prefered_action[2];
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==0)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==1)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==1))
			{
			state_mechine = 3;
			System.out.println("state_3");
			ACTION.state = 2;//prefered_action[3];
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==1)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==0)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==0))
			{
			state_mechine = 4;
			System.out.println("state_4");
			ACTION.state = 0;//prefered_action[4];
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==1)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==0)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==1))
			{
			state_mechine = 5;
			System.out.println("state_5");
			ACTION.state = 1;//prefered_action[5];;
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==1)&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==1)&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==0))
			{
			state_mechine = 6;
			System.out.println("state_6");
			ACTION.state = 0;//prefered_action[6];
			}
		if((PF_CLOSE_TO_HOMEBASE0_taekstep.intValue(curr_time)==1)
				&&(PF_TARGET0_IN_GRIPPER_takestep.intValue(curr_time)==1)
				&&(PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==1))
			{
			state_mechine = 7;
			System.out.println("state_7");
			ACTION.state = 1;//prefered_action[7];
			}
		
		count = count +1;
		//rn_now = rn_now*gamma;
		//rn = gamma*rn_now + rn;
		
		if(state_mechine==7)
		{
			System.out.println("7777777777777777777777777777777777777777777777777777777777777777777777777777777777777777");
		}
		
		if(remain != state_mechine)
		{
			//=============================================
			//寻找下一状态的动作
			//=============================================
	        for (int t = 0; t < 8; t++)
	        {
	        	Vmax [t] = -9999999999f;
	        }
	        int rd_q_action;
	        for(int j = 0; j < 8; j++)
	        {
	        for (int i = 0; i < 3; i++)
	                {
	                if (q[j][i] > Vmax [j])
	                        {
	                        Vmax [j] = q[j][i];
	                        max_q_action = i;
	                        }
	                }
	        double rd=Math.random();
	        Random random = new Random();
	        rd_q_action = random.nextInt(3)%(3) + 1;//random.nextInt(max)%(max-min+1) + min;
	        if (rd<exploration_rate)
	        {
	        	prefered_action[j] = rd_q_action - 1;
	        }
	        else
	        {
	        	prefered_action[j] = max_q_action;
	        }
	        }
	        
	   //=================================================================================================================================================
	        
		System.out.println("remain "+ remain);
		if ((remain!=6) && (remain!=7))
		{
			if ((state_mechine==6)||(state_mechine==7))
			{
				rn = 10;
				System.out.println("good!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				
				File file2 = new File("d:\\capture.txt"); //这里也要改
				FileWriter out = null;
				try {
					out = new FileWriter(file2,true);
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

					    try {
							out.write(SimulationCanvas.cishu +"\t");
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					    try {
							out.write(curr_time/1000 +"\t");
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					    try {
							out.write(count_cap +"\t");
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}

					    try {
							out.write(";" +"\t");
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					   try {
						out.write("\r\n");
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				
					  try {
						out.close();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					  count_cap = count_cap +1;
				
			}
			else 
			{
				rn = -0.02 * count;
			}
		}
		else 
		{
			rn = -0.02 * count;
		}
			
			/*
	        if (Math.abs(PS_GLOBAL_POS_taekstep.Value(curr_time).r)>0.5 
	        		&& Math.abs(PS_GLOBAL_POS_taekstep.Value(curr_time).r)<1.0
	        		&& AttractorSim.jieshu == true)
	        {
	        	rn_now = -2;
	        }
	        if (Math.abs(PS_GLOBAL_POS_taekstep.Value(curr_time).r)>1.0 
	        		&& Math.abs(PS_GLOBAL_POS_taekstep.Value(curr_time).r)<3.0
	        		&& AttractorSim.jieshu == true)
	        {
	        	rn_now = -3;
	        }
	        if (Math.abs(PS_GLOBAL_POS_taekstep.Value(curr_time).r)>3.0 
	        		&& AttractorSim.jieshu == true)
	        {
	        	rn_now = -5;
	        }
	        */
			
			q[ls][la] = (1 - alpha)*q[ls][la] +
                	alpha*(rn + gamma*Vmax[state_mechine]);
			ls = state_mechine;
			la = prefered_action[state_mechine];//ACTION.state;
			count = 1;
		//===========================================================================================
	    //将q矩阵写入文件
		//===========================================================================================
			File file = new File("d:\\q.txt"); 
			FileWriter out = null;
			try {
				out = new FileWriter(file);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			for(int i=0;i<8;i++)
			{
				for(int j=0;j<3;j++)
				{
				    try {
						out.write(q[i][j]+"\t");
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				 }
				   try {
					out.write("\r\n");
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
				  try {
					out.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
		}
		
		remain = state_mechine;
		
		
		//ACTION.follow_on[1][0] = SimulationCanvas.zero;
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

		// STATE DISPLAY
		int state = prefered_action[state_mechine];//ACTION.Value(curr_time);
		//int state = ACTION;
		if (state == 0)
			abstract_robot.setDisplayString("wander");
		else if (state == 1)
			abstract_robot.setDisplayString("acquire");
		else if (state == 2)
			abstract_robot.setDisplayString("deliver");
 
		return(CSSTAT_OK);
		}
	
	
}

