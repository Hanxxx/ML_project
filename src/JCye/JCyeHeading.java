/*
 * JCyeHeading.java
 */

package JCye;

import java.io.*;
import EDU.cmu.cs.coral.cye.*;

/**
 * 
 * Simple example program to control Cye heading from Java.
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

public class JCyeHeading 
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
        	Srv.Wait(100); // 100 ms
        	Srv.SendBuzzerOn(false);

		/*--- get heading and speed params from command line ---*/
        	Double Heading = new Double(args[0]);
        	Integer Velocity = new Integer(args[1]);

		/*--- send command to the robot ---*/
        	Srv.SendHeadingDestination(Heading.doubleValue(), 
			Velocity.intValue());

		//System.exit(0);
	    	}
	}
