/*
 * JCyeSquare.java
 */

package JCye;

import java.io.*;
import EDU.cmu.cs.coral.cye.*;

/**
 * 
 * Simple example program to drive the Cye in a square.
 * 
 * usage:   java JCyeHeading heading speed
 * example: java JCyeHeading 3.14 1000
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

public class JCyeSquare 
	{
    	public static void main(String args[]) 
		{
		boolean collision = false;

		/*--- initiate connection to robot ---*/
        	JCyeSrv Srv = new JCyeSrv(
			//--- comm device, uncomment one:
			"/dev/ttyS0",  // Linux
			//"COM1",          // Windows

			//--- baud rate, uncomment one:
			9600,
			//19200, 

			//--- link type, uncomment one:
			//JCyeComm.NEW_RADIO,   // connection is wired or radio
			JCyeComm.WIRED, // connection is wired or radio

			//--- robot ID, depends on color of robot
			JCyeComm.ORIGINAL_ROBOT); 

		/*--- make robot beep ---*/
        	Srv.SendBuzzerOn(true);
        	Srv.Wait(100); // 100 ms
        	Srv.SendBuzzerOn(false);

		/*--- reset robot ---*/
        	Srv.SendHeading(0);
		Srv.SendPosition(0,0);

		/*--- loop in a square ---*/
		while (!collision)
			{
		System.out.println("going straight");
		while ((Srv.GetLastX() < 60)&&(!collision))
			{
        		Srv.SendPositionVelocityDestination(80,0,300);
			System.out.println(Srv.GetLastX());
			System.out.println(Srv.GetLastY());
			System.out.println("----");
			Srv.Wait(200);
			collision = Srv.GetObstacle();
			}

		System.out.println("turning left");
		while ((Srv.GetLastY() < 60)&&(!collision))
			{
        		Srv.SendPositionVelocityDestination(80,80,300);
			System.out.println(Srv.GetLastX());
			System.out.println(Srv.GetLastY());
			System.out.println("----");
			Srv.Wait(200);
			collision = Srv.GetObstacle();
			}

		System.out.println("turning left");
		while ((Srv.GetLastX() > 20)&&(!collision))
			{
        		Srv.SendPositionVelocityDestination(0,80,300);
			System.out.println(Srv.GetLastX());
			System.out.println(Srv.GetLastY());
			System.out.println("----");
			Srv.Wait(200);
			collision = Srv.GetObstacle();
			}

		System.out.println("turning left");
		while ((Srv.GetLastY() > 20)&&(!collision))
			{
        		Srv.SendPositionVelocityDestination(0,0,300);
			System.out.println(Srv.GetLastX());
			System.out.println(Srv.GetLastY());
			System.out.println("----");
			Srv.Wait(200);
			collision = Srv.GetObstacle();
			}
			}

		System.exit(0);
	    	}
	}
