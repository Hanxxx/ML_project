package forage;
/*
 * forage.java
 */

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.simulation.SquiggleBallSim;
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

public class multiforage_s  extends ControlSystemMFN150
	{
	public final static boolean DEBUG = true;
	private NodeVec2	turret_configuration;
	private NodeVec2	steering_configuration;
	private NodeDouble	gripper_fingers_configuration;
	private NodeDouble	gripper_height_configuration;
	private NodeBoolean PF_TARGET0_VISIBLE_takestep;
	private NodeInt 	state_monitor;
	private i_FSA_ba STATE_MACHINE;
	private NodeVec2 SquiggleBall;
	public static Vec2 SquiggleBall_direction_s ;
	public static int judge_s;
	public static int oncespotted_s = 0;
	private NodeVec2 fangxiang;
	public static double dir_s = 0; 
	public static Vec2 position_0_s;
	public static int weibu_s;
	//=======================================================================
	private double  q[][];                  // the q-values
	private double  gamma=0.8;              // discount rate
	private double  alpha=0.2;              // learning rate
	private Random  rgen;                   // the random number generator
	private int     ls = 0;                     // last state
	private int     la = 0;                     // last action
	private double  exploration_rate = 0.2; // 
	private long	seed;			// random number seed
	private int     remain = 0;
	private double     rn1 = 0;                     //reward
	private double     rn = 0;   
	private double     rn_now ;  
    private double  Vmax;  //very bad
    private int max_q_action = 0;
    private int prefered_action;
    private int in_what_state = 0;
    private Vec2 incentre_dis;
    private Vec2 person_dis;
    private double xcor = 0;
    private double ycor = 0;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		
		oncespotted_s = 0;
		rn = 0; 
		weibu_s =0;
		//======
		// Set some initial hardware configurations.
		//======
		abstract_robot.setObstacleMaxRange(0.5); // don't consider   //1.0
	 						 // things further away
		abstract_robot.setBaseSpeed(1.0*abstract_robot.MAX_TRANSLATION);// *0.8
		abstract_robot.setGripperHeight(-1,0);   // put gripper down
		abstract_robot.setGripperFingers(-1,1);  // open gripper

		
		//======
		// perceptual schemas
		//======
		//--- robot's global position
		NodeVec2
		PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);
		SquiggleBall = PS_GLOBAL_POS;

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
		PS_TARGETS0_GLOBAL_FILT = new va_FilterOutClose_vva(0.75,
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

		// close to homebase 
		NodeBoolean
		PF_CLOSE_TO_HOMEBASE0 = new b_Close_vv(0.4, PS_GLOBAL_POS,
			PS_HOMEBASE0_GLOBAL);


		//======
		// motor schemas
		//======
		// avoid obstacles
		NodeVec2
		MS_AVOID_OBSTACLES = new v_Avoid_va( 0.5 ,//原来是1.5   //0.5
				abstract_robot.RADIUS + 0.01,//abstract_robot.RADIUS + 0.01
			PS_OBS);

		// swirl obstacles wrt target 0
		NodeVec2
		MS_SWIRL_OBSTACLES_TARGET0 = new v_Swirl_vav(0.5,//原来2.0   //0.3
				abstract_robot.RADIUS + 0.01,//原来0.22//abstract_robot.RADIUS + 0.01
			PS_OBS,
			PS_CLOSEST0);

		// swirl obstacles wrt homebase0
		NodeVec2
		MS_SWIRL_OBSTACLES_HOMEBASE0 = new v_Swirl_vav(0.5,//原来2.0
				abstract_robot.RADIUS + 0.01,//原来0.22//abstract_robot.RADIUS + 0.01
			PS_OBS,
			PS_HOMEBASE0);

		// go home 0
		NodeVec2
		MS_MOVE_TO_HOMEBASE0 = new v_LinearAttraction_v(1.0,0.0,PS_HOMEBASE0);//原来第一项0.4

		// go to target0
		NodeVec2
		MS_MOVE_TO_TARGET0 = new v_LinearAttraction_v(1.0,0.0,PS_CLOSEST0);//原来第一项0.4
		
		fangxiang = new wojiade_s(1.0,0.0,SquiggleBallSim.SquiggleBalldircection);

		// noise vector
		NodeVec2
		MS_NOISE_VECTOR = new v_Noise_(5,seed);

		// swirl obstacles wrt noise
		NodeVec2
		MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(0.5,//原来2.0
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
		AS_ACQUIRE0.embedded[1] = MS_SWIRL_OBSTACLES_TARGET0;

		AS_ACQUIRE0.weights[2]  = 0;  //原来是0.2
		AS_ACQUIRE0.embedded[2] = MS_NOISE_VECTOR;
		
		AS_ACQUIRE0.weights[3]  = 1.0;
		AS_ACQUIRE0.embedded[3] = fangxiang;



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

		AS_DELIVER0.weights[3]  = 0.2;
		AS_DELIVER0.embedded[3] = MS_NOISE_VECTOR;



		//======
		// STATE_MACHINE
		//======
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;
		
		// STATE 0 WANDER
		STATE_MACHINE.triggers[0][0]  = PF_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[0][0] = 1; // transition to ACQUIRE0

		// STATE 1 ACQUIRE0
		STATE_MACHINE.triggers[1][0]  = PF_TARGET0_IN_GRIPPER;
		STATE_MACHINE.follow_on[1][0] = 2; // transition to DELIVER   
		STATE_MACHINE.triggers[1][1]  = PF_NOT_TARGET0_VISIBLE;
		STATE_MACHINE.follow_on[1][1] = 1; // transition to WANDER   原来是 0

		// STATE 2 DELIVER0
		STATE_MACHINE.triggers[2][0]  = PF_CLOSE_TO_HOMEBASE0;
		STATE_MACHINE.follow_on[2][0] = 0; // transition to WANDER

		state_monitor = STATE_MACHINE;


		//======
		// STEERING
		//======
		v_Select_vai
		STEERING = new v_Select_vai((NodeInt)STATE_MACHINE);

		STEERING.embedded[0] = AS_WANDER;
		STEERING.embedded[1] = AS_ACQUIRE0;
		STEERING.embedded[2] = AS_DELIVER0;


		//======
		// TURRET
		//======
		v_Select_vai
		TURRET = new v_Select_vai((NodeInt)STATE_MACHINE);

		TURRET.embedded[0] = AS_WANDER;
		TURRET.embedded[1] = AS_ACQUIRE0;
		TURRET.embedded[2] = AS_DELIVER0;


		//======
		// GRIPPER_FINGERS
		//======
		d_Select_i
		GRIPPER_FINGERS = new d_Select_i(STATE_MACHINE);

		GRIPPER_FINGERS.embedded[0] = 1;  // open in WANDER
		GRIPPER_FINGERS.embedded[1] = -1; // trigger in ACQUIRE //  -1
		GRIPPER_FINGERS.embedded[2] = 0;  // closed in DELIVER


		turret_configuration = TURRET;
		steering_configuration = STEERING;
		gripper_fingers_configuration = GRIPPER_FINGERS;
		
		/*
		//======================================================================
		//初始化Q值表
		//======================================================================
		
		rgen = new Random(seed);
		q = new double[32][5];
        for(int i=0; i<32; i++)
        {
        	for(int j=0; j<5; j++)
        	{
        		q[i][j] = rgen.nextDouble()*2 - 1;
        	}
        }
        
		//======================================================================
		//将Q值表写入文件
		//======================================================================
        File file = new File("d:\\Q4_origin.txt"); 
		FileWriter out = null;
		try {
			out = new FileWriter(file);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		for(int i=0;i<32;i++)
		{
			for(int j=0;j<5;j++)
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
		
		*/
		//==========================================================================
		//从文件中读入数据
		//==========================================================================
				double[][] arr2 = new double[32][5];
				q = new double[32][5];
				File file = new File("d:\\Q4.txt"); 
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
		        
		   la = 0;
		   ls = 0;
		   
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
		int a=0;
		int b=0;
		int c=0;
		int d=0;
		double G_theta = 0;
		double J = 0;
		double dis_sum = 0;
		Vec2 person_to_SquiggleBall;
   	    person_to_SquiggleBall = new Vec2 (SquiggleBallSim.SquiggleBalldircection.x-SquiggleBall.Value(curr_time).x,SquiggleBallSim.SquiggleBalldircection.y-SquiggleBall.Value(curr_time).y);
   	    double theta_m = 0;
		int judg;
	    int index_this=0;
	    double left=0;
	    double right=0;
	    
	    //weibu_s = 0;
		judg =0;
		judge_s = 0;
		

		// STEER
		//==============================================================================
		//选择要执行的动作
		//==============================================================================
		
		//判断所处状态
		
		//首先计算内心坐标   ///四个的时候是重心
		
		if (curr_time >100)
		{
			oncespotted_s = 1;
		
			person_dis = new Vec2 (SquiggleBallSim.SquiggleBalldircection.x-SquiggleBall.Value(curr_time).x,SquiggleBallSim.SquiggleBalldircection.y -SquiggleBall.Value(curr_time).y);

			if (Math.abs(person_dis.r)<=0.5)
			{
				a = 0;
			}
			if ((Math.abs(person_dis.r)>0.5)&&(Math.abs(person_dis.r)<=1.4))
			{
				a = 1;
			}
			if ((Math.abs(person_dis.r)>1.4)&&(Math.abs(person_dis.r)<=5.0))
			{
				a = 2;
			}
			if (Math.abs(person_dis.r)>5.0)
			{
				a = 3;
			}
			
			for (int i=0; i<8; i++)
			{
				if (person_dis.t<0)
				{
					person_dis.t = person_dis.t + Math.PI*2;
				}
				if ((person_dis.t>=i*(Math.PI/4)) && (person_dis.t<(i+1)*(Math.PI/4)))
				{
					b =i;
				}
			}

			in_what_state = 8*a + b;  		  
			
			//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        	Vec2 SA = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s.position_0_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s.position_0_s.y);
        	Vec2 SB = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s1.position_1_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s1.position_1_s.y);
        	Vec2 SC = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s2.position_2_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s2.position_2_s.y);
        	Vec2 SD = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s3.position_3_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s3.position_3_s.y);

        	double[] theta = new double[4];
            int[] index =new int[4];
            index[0] = 0;
            index[1] = 1;
            index[2] = 2;
            index[3] = 3;
        	theta[0] = SA.t;
        	if (theta[0]<0)
        	{
        		theta[0] = theta[0]+Math.PI*2;
        	}
        	theta[1] = SB.t;
        	if (theta[1]<0)
        	{
        		theta[1] = theta[1]+Math.PI*2;
        	}
        	theta[2] = SC.t;
        	if (theta[2]<0)
        	{
        		theta[2] = theta[2]+Math.PI*2;
        	}
        	theta[3] = SD.t;
        	if (theta[3]<0)
        	{
        		theta[3] = theta[3]+Math.PI*2;
        	}
        	//排序[0]最大
		       double temp_count; // 记录临时中间值   
		       int temp_count1;
		       int size = 4; // 数组大小   
		       for (int i = 0; i < size-1; i++) 
		       {   
		           for (int j = i + 1; j < size; j++) 
		           {   
		               if (theta[i] < theta[j])
		               { // 交换两数的位置   
		            	   temp_count = theta[i];   
		                   theta[i] = theta[j];   
		                   theta[j] = temp_count;
		            	   temp_count1 = index[i];   
		                   index[i] = index[j];   
		                   index[j] = temp_count1;
		               }   
		           }   
		       }

        	for (int i=0; i<4;i++)
        	{
        		if (index[i]==0)//改这里
        		{
        			index_this = i;
        		}
        	}
		    double[] theta_b = new double[4];//四边形中间一点的夹角
		       
		    theta_b[0] = theta[0] - theta[1];
		    theta_b[1] = theta[1] - theta[2];
		    theta_b[2] = theta[2] - theta[3];
		    theta_b[3] = theta[3] - theta[0];
		    if (theta_b[3]<0)
		    {
		    	theta_b[3] = theta_b[3]+Math.PI*2;
		    }
			    
		    
		    double theta_max  = -9999999999f;
			for (int i=0; i<4; i++)
			{
				if (theta_b[i] > theta_max)
                {
					theta_max  = theta_b[i];
                }
			}
			theta_m = theta_max;
			
		}

	
		if (multiforage_s.oncespotted_s != 1)
		{
			remain = in_what_state;
			la = 0;
			ls = in_what_state;
		}
		
		//Q学习过程
		
		if (multiforage_s.oncespotted_s == 1)
		{
        	Vec2 SA = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s.position_0_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s.position_0_s.y);
        	Vec2 SB = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s1.position_1_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s1.position_1_s.y);
        	Vec2 SC = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s2.position_2_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s2.position_2_s.y);
        	Vec2 SD = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-multiforage_s3.position_3_s.x,SquiggleBallSim.SquiggleBalldircection.y-multiforage_s3.position_3_s.y);

	        	if ((Math.abs(SA.r)<1.4)&&(Math.abs(SB.r)<1.4)&&(Math.abs(SC.r)<1.4)&&(Math.abs(SD.r)<1.4))
	        	{
	        		judg = 1;
	        	}
			if(remain != in_what_state)
			{
				//==========================================================================
				//从文件中读入数据
				//==========================================================================
						double[][] arr2 = new double[32][5];
						q = new double[32][5];
						File file = new File("d:\\Q4.txt"); 
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
				//状态选择
				Vmax  = -9999999999f;
				for (int i=0; i<5; i++)
				{
					if (q[in_what_state][i] > Vmax)
	                {
	                Vmax  = q[in_what_state][i];
	                max_q_action = i;
	                }
			        double rd=Math.random();
			        Random random = new Random();
			        int rd_q_action = random.nextInt(5)%(5) + 1;//random.nextInt(max)%(max-min+1) + min;
			        if (rd<exploration_rate)
			        {
			        	prefered_action = rd_q_action - 1;
			        }
			        else
			        {
			        	prefered_action = max_q_action;
			        }
			       

				}
				
		        //将选择的动作实现
		        if (prefered_action==0)
		        {
		        	dir_s = person_to_SquiggleBall.t;
		        }
		        if (prefered_action==1)
		        {
		        	dir_s = person_to_SquiggleBall.t + (Math.PI/4);
		        }
		        if (prefered_action==2)
		        {
		        	dir_s = person_to_SquiggleBall.t - (Math.PI/4);
		        }
		        if (prefered_action==3)
		        {
		        	dir_s = person_to_SquiggleBall.t + (Math.PI/2);
		        }
		        if (prefered_action==4)
		        {
		        	dir_s = person_to_SquiggleBall.t - (Math.PI/2);
		        }

			//==============================================================
			//奖赏函数与Q值更新
			//==============================================================
		    //设定奖赏函数
				if (Math.abs(person_dis.r)<=0.5)
				{
					rn_now = 2.0;
				}
				if ((Math.abs(person_dis.r)>0.5)&&(Math.abs(person_dis.r)<=1.4))
				{
					rn_now = 1.5;
				}
				if ((Math.abs(person_dis.r)>1.4)&&(Math.abs(person_dis.r)<=5.0))
				{
					rn_now = 0.5;
				}
				if (Math.abs(person_dis.r)>5.0)
				{
					rn_now = 0;
				}
				
		        
			q[ls][la] = (1 - alpha)*q[ls][la] +
                	alpha*(rn + gamma*Vmax);


			ls = in_what_state;
			la = prefered_action;
			rn = rn_now;
			
				//===========================================================================================
			    //将q矩阵写入文件
				//===========================================================================================
					File file1 = new File("d:\\Q4.txt"); 
					FileWriter out = null;
					try {
						out = new FileWriter(file1);
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					for(int i=0;i<32;i++)
					{
						for(int j=0;j<5;j++)
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
						  remain =remain;
						  
							if ((judg==1)&&(Math.abs(theta_m-Math.PI/2)<Math.PI*0.111))
							{
								weibu_s = 1;
								rn = 10;
							}
		     }

		
		remain = in_what_state;
		}
		
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, result.r);
		
		//得到现在的方向
		dir_s = result.t;

		// TURRET
		result = turret_configuration.Value(curr_time);
        if (multiforage_s.oncespotted_s == 1)
        {
        	Vec2 aa = new Vec2(SquiggleBallSim.SquiggleBalldircection.x-SquiggleBall.Value(curr_time).x,SquiggleBallSim.SquiggleBalldircection.y-SquiggleBall.Value(curr_time).y);
        	abstract_robot.setTurretHeading(curr_time, aa.t);
        }
        else
        {
		abstract_robot.setTurretHeading(curr_time, result.t);
        }

		// FINGERS
		dresult = gripper_fingers_configuration.Value(curr_time);
		abstract_robot.setGripperFingers(curr_time, dresult);

		
		SquiggleBall_direction_s = SquiggleBall.Value(curr_time);
		position_0_s = SquiggleBall.Value(curr_time);
		
		 //System.out.println(oncespotted_s);
		 //一但发现目标则turret朝着目标
		 if (PF_TARGET0_VISIBLE_takestep.intValue(curr_time)==1)
		 {
			 judge_s = 1;
		 }
			  
		
		// STATE DISPLAY
	     int state = STATE_MACHINE.Value(curr_time);
		//有一个发现则都进入acquire状态
		if (oncespotted_s == 1)
		{
			state = 1;
			STATE_MACHINE.state = 1;
		}
		if (state == 0)
		{
			abstract_robot.setDisplayString("wander");
			
		}
		else if (state == 1)
		{
			abstract_robot.setDisplayString("acquire");
			
			oncespotted_s = 1;
			

		}
		else if (state == 2)
		{
			abstract_robot.setDisplayString("deliver");
			judge_s = 2;
		}

		return(CSSTAT_OK);
		}
	}
