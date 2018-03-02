/*
 * TBHard.java
 */

package TBHard;

import java.io.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.*;
import EDU.cmu.cs.coral.abstractrobot.*;

/**
 * Runs a control system on a Nomad 150 and Cye(and soon other hardware).
 * It intializes the robot,
 * then calls the control system periodically to
 * drive the robot.
 * <P>
 * To run this program, first ensure you are in the correct directory (where
 * the TBHard.class file is), then type "java TBHard".
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997, 1998 Tucker Balch
 *
 * @author Tucker Balch
 * @version $Revision: 1.3 $
 * @see EDU.gatech.cc.is.abstractrobot.CommN150Hard
 */

public class TBHard
	{

        /**
	 * Main for TBHard.
         */
	public static void main(String[] args)
		{
		SimpleInterface abstract_robot = null;
		ControlSystemS cs = new ControlSystemS();
		boolean keep_going = true;
		int runtime = 0;
		double	startx = 0;
		double	starty = 0;
		String  server;
		int id = 0;
		String robotType;

		/*--- check the arguments first ---*/
		if (args.length >= 1)
			{
			if (args[0].equalsIgnoreCase("-version"))
                               {
                               System.out.println(
                                        TBVersion.longReport());
                                System.exit(0);
                               }
			}
		System.out.println(TBVersion.shortReport());
		if (args.length < 7)
			{
			System.out.println("usage: java TBHard [-version] controlsystem commserver id secondstorun x y robottype");
			System.exit(1);
			}

		/*--- get server ---*/
		server = args[1];
	
		/*--- check id ---*/
		try
			{
			id = Integer.valueOf(args[2]).intValue();
			}
		catch (NumberFormatException e)
			{
			System.out.println(args[2] + " is not a valid id.");
			System.exit(1);
			}
		if (id <= 0)
			{
			System.out.println(args[2] + " is not a valid id.");
			System.exit(1);
			}

		/*--- check run time ---*/
		try
			{
			runtime = 1000*Integer.valueOf(args[3]).intValue();
			}
		catch (NumberFormatException e)
			{
			System.out.println(args[3] + " is not a valid number of seconds.");
			System.exit(1);
			}
		if (runtime <= 0)
			{
			System.out.println(args[3] + " is not a valid number of seconds.");
			System.exit(1);
			}

		/*--- get the starting position ---*/
		try
			{
			startx = Double.valueOf(args[4]).doubleValue();
			starty = Double.valueOf(args[5]).doubleValue();
			}
		catch (NumberFormatException e)
			{
			System.out.println("("+args[4]+","+args[5]+")"
				+" is not a valid starting position.");
			System.exit(1);
			}
		robotType = args[6];
		if (!robotType.equals("N") && !robotType.equals("C"))
		  {
		    System.out.println("Only N(omad 150) or C(ye) are valid robot types");
		    System.exit(1);
		  }

		/*--- load the control system ---*/
		try
			{
			Class csclass = Class.forName(args[0]);
			if (robotType.equals("N"))
			  {
			  try {
			     cs = (ControlSystemMFN150)csclass.newInstance();
			} catch (ClassCastException e)
			  {
			    System.out.println(args[0]+ " is not of type ControlSystemMFN150");
			    System.exit(1);
			  }
			  }
			else
			  {
			   try { 
			     cs = (ControlSystemCye)csclass.newInstance();
			   } catch (ClassCastException e)
			     {
			       System.out.println(args[0]+ " is not of type ControlSystemCye");
			       System.exit(1);
			     }
			  }
			}
		catch (ClassNotFoundException e )
			{
			System.out.println(args[0]+
				" was not found.");
			System.exit(1);
			}
		catch (IllegalAccessException e)
			{
			System.out.println(args[0]+
				" is not a ControlSystemN150 or ControlSystemCye object.");
			System.exit(1);
			}
		catch (InstantiationException e)
			{
			System.out.println(args[0]+
				" could not be instantiated.");
			System.exit(1);
			}

		try
			{
			/*--- inform the user ---*/
			System.out.println("TBHard: initializing the robot.");
			/*--- open the connection to the robot hardware ---*/
			try
				{
				if (robotType.equals("N"))
				    abstract_robot = new 
					MultiForageN150HardPassiveGrip(
						1,38400);
					//MultiForageN150HardPassiveGrip(
						//1,38400,server,id);
				//abstract_robot = new CommN150Hard(1,38400);
				 else
				    abstract_robot = new 
					SimpleCyeHard("/dev/ttyS0", 19200, 
						      "W", 1);
				}
			catch(Exception e)
				{
				System.out.println(e);
				}
			if (robotType.equals("N"))
			  {
			  ((MultiForageN150HardPassiveGrip)abstract_robot).setBaseSpeed(((MultiForageN150HardPassiveGrip)abstract_robot).MAX_TRANSLATION);
			 abstract_robot.resetPosition(new Vec2(startx, starty));
			cs.init((MultiForageN150HardPassiveGrip)abstract_robot,System.currentTimeMillis());
			  }
			else
			  {
			  ((SimpleCyeHard)abstract_robot).setBaseSpeed(((SimpleCyeHard)abstract_robot).MAX_TRANSLATION);
			 abstract_robot.resetPosition(new Vec2(startx, starty));
			cs.init((SimpleCyeHard)abstract_robot,System.currentTimeMillis());
			  }

			/*--- initialize the control system ---*/
			cs.configure();
			cs.trialInit();
			}
		catch(Exception e)
			{
			System.out.println(e);
			System.exit(1);
			}

		/*--- wait to begin ---*/
		//System.out.println("TBHard: ready to go, hit return to start.");
		//int inchar = 0;
		//while(inchar != 10)
			//{
			//try{inchar = System.in.read();}
			//catch(Exception e){}
			//}

		/*--- get the time we started ---*/
		long start_time = abstract_robot.getTime();
		long curr_time = start_time;
		double cycles = 0;

		/*--- loop until done ---*/
		int ret_val = ControlSystemS.CSSTAT_OK;
		System.out.println("******STARTING******");
		while (((curr_time - start_time)<runtime)&&
			(ret_val == ControlSystemS.CSSTAT_OK))
			{
			// think about what to do
			ret_val = cs.takeStep();

			// let the hardware run
			if (robotType.equals("N"))
			   ((MultiForageN150HardPassiveGrip)abstract_robot).takeStep();
			else
			   ((SimpleCyeHard)abstract_robot).takeStep();

			// let other threads run
			// NOPE! it runs synchronously now
			//try{Thread.sleep(50);}
			//catch(Exception e){}

			// garbage collect (for uniform cycle times)
			System.gc();

			// keep track of time
			cycles++;
			curr_time = abstract_robot.getTime();
			}
		curr_time = abstract_robot.getTime();
		System.out.println("******STOPPING******");

		/*--- shutdown ---*/
		cs.trialEnd();
		cs.quit();
		abstract_robot.quit(); // only do this when done!
		System.out.println("TBHard: "+
			(cycles*1000/(double)curr_time)+ 
			" control cycles per second.");

		}
/*
	catch(Exception e)
		{
		System.out.println(e);
		System.exit(1);
		}
*/
	}
