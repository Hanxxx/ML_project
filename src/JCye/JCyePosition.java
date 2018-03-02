/*
 * JCyePosition
 */

package JCye;

import java.io.*;
import EDU.cmu.cs.coral.cye.*;

/**
 *
 * Simple example program to move the Cye to a new position.
 *
 * usage:   java JCyePosition X Y speed
 * example: java JCyePosition 100 100 1000
 *
 * <P>
 * <A HREF="../Docs/copyright.html">Copyright</A>
 * (c)2000 Brian Chemel and Tucker Balch
 *
 * @author Brian Chemel and Tucker Balch
 *
 * @see EDU.cmu.cs.coral.cye.JCyeSrv
 * @see EDU.cmu.cs.coral.cye.JCyeMsg
 * @see EDU.cmu.cs.coral.cye.JCyeStatus
 */

public class JCyePosition
	{
	public static void main(String args[]) 
		{
		/*--- initiate connection to robot ---*/
		JCyeSrv Srv = new JCyeSrv(
			//--- comm device, uncomment one:
			//"/dev/ttyS0",  // Linux
			"COM1",          // Windows

			//--- baud rate, uncomment one:
			//9600,
			19200, 

			//--- link type, uncomment one:
			JCyeComm.NEW_RADIO,   // connection is wired or radio
			//JCyeComm.WIRED, // connection is wired or radio

			//--- robot ID, depends on color of robot
			JCyeComm.ORIGINAL_ROBOT); 

		/*--- make robot beep ---*/
		Srv.SendBuzzerOn(true);
		Srv.Wait(100);
		Srv.SendBuzzerOn(false);

		/*--- get params from command line ---*/
		Double X = new Double(args[0]);
		Double Y = new Double(args[1]);
		Integer Velocity = new Integer(args[2]);

		/*--- set handle length ---*/
		Srv.SendHandleLength(150);

		/*--- loop until reach destination ---*/
		boolean interrupted = false;
		while(!interrupted) 
			{
	    		Srv.Wait(50);
			Srv.SendPositionVelocityDestination(X.doubleValue(), 
				Y.doubleValue(), Velocity.intValue());
	    		System.out.println("Last X: " + Srv.GetLastX());
	    		System.out.println("Last Y: " + Srv.GetLastY());
	    		System.out.println("Last H: " + Srv.GetLastH());
	    		System.out.println("Last B: " + Srv.GetLastB());
	    		System.out.println();

	    		interrupted = Srv.GetObstacle();

	    		if(interrupted) 
				System.out.println("ran into obstacle");
			}
		Srv.SendStopMotors();

		System.out.println("Last X: " + Srv.GetLastX());
		System.out.println("Last Y: " + Srv.GetLastY());
		System.out.println("Last H: " + Srv.GetLastH());
		System.out.println("Last B: " + Srv.GetLastB());

		System.exit(0);
		}
	}
