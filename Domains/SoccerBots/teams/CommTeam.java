/*
 * CommTeam.java
 */

import	java.util.Enumeration;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.communication.*;
//Clay not used

/**
 * Demonstration of communication in a simple robot soccer team.
 * The agents don't actually use the communication, they just
 * send an occasional message back and forth.  
 * All the robots listen for incoming messages, but only robot
 * number three actually sends them.  Look for COMMUNICATION
 * tags in the text indicating the communication demonstration.
 * Look at Transceiver in the communication package for more details.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1998 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */


public class CommTeam extends ControlSystemSS
	{
	/**
	 * This enumeration buffers incoming messages.
	 */
	private Enumeration messagesin;	//COMMUNICATION

	/**
	Configure the control system.  This method is
	called once at initialization time.  You can use it
	to do whatever you like.
	*/
	public void Configure()
		{
		/*--- Instantiate the message buffer ----*/
		messagesin = abstract_robot.getReceiveChannel();//COMMUNICATION
		}
		
	
	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int TakeStep()
		{
		// the eventual movement command is placed here
		Vec2	result = new Vec2(0,0);

		// get the current time for timestamps
		long	curr_time = abstract_robot.getTime();


		/*--- Get some sensor data ---*/
		// get vector to the ball
		Vec2 ball = abstract_robot.getBall(curr_time);

		// get vector to our and their goal
		Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
		Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);

		// get a list of the positions of our teammates
		Vec2[] teammates = abstract_robot.getTeammates(curr_time);

		// find the closest teammate
		Vec2 closestteammate = new Vec2(99999,0);
		for (int i=0; i< teammates.length; i++)
			{
			if (teammates[i].r < closestteammate.r)
				closestteammate = teammates[i];
			}


		/*--- now compute some strategic places to go ---*/
		// compute a point one robot radius
		// behind the ball.
		Vec2 kickspot = new Vec2(ball.x, ball.y);
		kickspot.sub(theirgoal);
		kickspot.setr(abstract_robot.RADIUS);
		kickspot.add(ball);

		// compute a point three robot radii
		// behind the ball.
		Vec2 backspot = new Vec2(ball.x, ball.y);
		backspot.sub(theirgoal);
		backspot.setr(abstract_robot.RADIUS*5);
		backspot.add(ball);

		// compute a north and south spot
		Vec2 northspot = new Vec2(backspot.x,backspot.y+0.7);
		Vec2 southspot = new Vec2(backspot.x,backspot.y-0.7);

		// compute a position between the ball and defended goal
		Vec2 goaliepos = new Vec2(ourgoal.x + ball.x,
				ourgoal.y + ball.y);
		goaliepos.setr(goaliepos.r*0.5);

		// a direction away from the closest teammate.
		Vec2 awayfromclosest = new Vec2(closestteammate.x,
				closestteammate.y);
		awayfromclosest.sett(awayfromclosest.t + Math.PI);


		/*--- go to one of the places depending on player num ---*/
		int mynum = abstract_robot.getPlayerNumber(curr_time);

		/*--- Goalie ---*/
		if (mynum == 0)
			{
			// go to the goalie position if far from the ball
			if (ball.r > 0.5) 
				result = goaliepos;
			// otherwise go to kick it
			else if (ball.r > 0.1) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		/*--- midback ---*/
		else if (mynum == 1)
			{
			// go to a midback position if far from the ball
			if (ball.r > 0.5) 
				result = backspot;
			// otherwise go to kick it
			else if (ball.r > 0.30) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		else if (mynum == 2)
			{
			// go to a the northspot position if far from the ball
			if (ball.r > 0.5) 
				result = northspot;
			// otherwise go to kick it
			else if (ball.r > 0.30) 
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		else if (mynum == 4)
			{
			// go to a the northspot position if far from the ball
			if (ball.r > 0.5) 
				result = southspot;
			// otherwise go to kick it
			else if (ball.r > 0.3 )
				result = kickspot;
			else 
				result = ball;
			// keep away from others
			if (closestteammate.r < 0.3)
				{
				result = awayfromclosest;
				}
			}

		/*---Lead Forward ---*/
		else if (mynum == 3)
			{
			// if we are more than 4cm away from the ball
			if (ball.r > .3)
				// go to a good kicking position
				result = kickspot;
			else
				// go to the ball
				result = ball;
			}


		/*--- Send commands to actuators ---*/
		// set the heading
		abstract_robot.setSteerHeading(curr_time, result.t);

		// set speed at maximum
		abstract_robot.setSpeed(curr_time, 1.0);

		// kick it if we can
		if (abstract_robot.canKick(curr_time))
			abstract_robot.kick(curr_time);

		/*--- Send message to other robot ---*/
		// COMMUNICATION
		// if I can kick
		if (abstract_robot.canKick(curr_time))
			{
			// and I am robot #3
			if ((mynum==3))
				{
				// construct a message
				StringMessage m = new StringMessage();
				m.val = (new Integer(mynum)).toString();
				m.val = m.val.concat(" can kick");

				// send the message to robot 2
				// note: broadcast and multicast are also
				// available.
				try{abstract_robot.unicast(2,m);}
				catch(CommunicationException e){}
				}
			}

		/*--- Look for incoming messages ---*/
		//COMMUNICATION
		while (messagesin.hasMoreElements())
			{
			StringMessage recvd = 
				(StringMessage)messagesin.nextElement();
			System.out.println(mynum + " received:\n" + recvd);
			}

		// tell the parent we're OK
		return(CSSTAT_OK);
		}
	}
