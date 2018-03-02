/*
 * DumbNodeGen.java
 */

import	java.io.*;
import	java.util.Random;
import	java.util.Enumeration;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	EDU.gatech.cc.is.communication.*;
import	EDU.gatech.cc.is.learning.*;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.cmu.cs.coral.abstractrobot.*;
import	EDU.cmu.cs.coral.util.*;

/**
 * A dumb network node.
 * <P>
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2000 CMU
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class DumbNodeGen  extends ControlSystemNetNode
	{
	public final static boolean DEBUG = false;
	protected Random rando;
	protected int[]    connected_to = new int[2];
	protected double[] distribution = new double[2];
	protected Vec2[] locations = new Vec2[2];
        private Enumeration messagesin; //to buffer incoming messages
	private int mynum = 0;


	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		/*--- init stuff ---*/
		for (int i=0; i<2; i++)
			locations[i] = new Vec2();

		rando = new Random(seed);

		/*--- get node number ---*/
	        mynum = abstract_robot.getPlayerNumber(-1);
		if (DEBUG) System.out.println(mynum + " configuring");

                /*--- set up topography ----*/
		if (mynum == 0)
			{
			connected_to[0] = 1;
			distribution[0] = 0.9;

			connected_to[1] = 2;
			distribution[1] = 0.1;
			
			// send the first message
                        StringMessage m = new StringMessage("I am the message");
			try
				{
				abstract_robot.unicast(2,m);
				}
			catch(CommunicationException e){}
			}
		else if (mynum == 1)
			{
			connected_to[0] = 2;
			distribution[0] = 0.9;

			connected_to[1] = 0;
			distribution[1] = 0.1;
			}
		else if (mynum == 2)
			{
			connected_to[0] = 0;
			distribution[0] = 0.9;

			connected_to[1] = 1;
			distribution[1] = 0.1;
			}

		/*--- tell everybody our location ---*/
		//PositionMessage p = new PositionMessage(
			//abstract_robot.getPosition(-1));
		//abstract_robot.broadcast(p);

                /*--- Instantiate the message buffer ----*/
                messagesin = abstract_robot.getReceiveChannel();//COMMUNICATION

		/*--- Read from dictionary ---*/
		TBDictionary dict = abstract_robot.getDictionary();
		String file = dict.getString("topology");
		System.out.println("topology in " + file);
		}
		

	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int takeStep()
		{
		/*--- get the time ---*/
                long    curr_time = abstract_robot.getTime();

		abstract_robot.setDisplayString(" ");

		/*--- while there are messages in the queue ---*/
		while (messagesin.hasMoreElements())
			{
			/*--- display it ---*/
			abstract_robot.setDisplayString("#");

			/*--- pull a message out of the queue ---*/
                        StringMessage recvd =
                                (StringMessage)messagesin.nextElement();
                        if (DEBUG) 
				System.out.println(mynum 
					+ " received  : " + recvd);

			/*--- who to send it to? ---*/
			double send = rando.nextDouble();
			int i = 0;
			for (i=0; i<connected_to.length; i ++)
				{
				send -= distribution[i];
				if (send<=0) break;
				}
			int to = connected_to[i];
                        if (DEBUG) 
				System.out.println(mynum 
					+ " sending to: " + to);

			/*--- send it ---*/
			try
				{
				abstract_robot.unicast(to,recvd);
				}
			catch(CommunicationException e){}
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
