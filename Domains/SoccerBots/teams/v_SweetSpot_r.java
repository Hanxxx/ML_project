/*
 * v_SweetSpot_r.java
 */

import java.lang.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.util.Vec2;

/**
 * Report the egocentric position of a good spot to kick from.
 * This module is a node used in Clay to configure soccer robot behavior.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997, 1998 Tucker Balch
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */


public class v_SweetSpot_r extends NodeVec2
	{
	/** 
	 * Turn debug printing on or off.
	 */
	public static final boolean DEBUG = Node.DEBUG;
	private SocSmall abstract_robot;

	/**
	 * Instantiate a v_SweetSpot_r schema.
	 * @param ar SocSmall, the abstract_robot object 
	 * that provides hardware support.
	 */
	public v_SweetSpot_r(SocSmall ar)
		{
		if (DEBUG) System.out.println("v_SweetSpot_r: instantiated");
		abstract_robot = ar;
		}

	long	last_spott = 0;
	Vec2	last_spot = new Vec2();
	/**
	 * Return a Vec2 pointing from the
	 * center of the robot to the sweet spot.
	 * @param timestamp long, only get new information 
	 *        if timestamp > than last call or timestamp == -1.
	 * @return the sensed ball
	 */
	public Vec2 Value(long timestamp)
		{
		if (DEBUG) System.out.println("v_SweetSpot_r: Value()");

		if ((timestamp > last_spott) || (timestamp == -1))
			{
			if (timestamp != -1) last_spott = timestamp;

			Vec2 ball = abstract_robot.getBall(timestamp);
			Vec2 goal = abstract_robot.getOpponentsGoal(timestamp);

			last_spot = new Vec2(ball.x, ball.y);
			last_spot.sub(goal);
			last_spot.setr(abstract_robot.RADIUS);
			last_spot.add(ball);
			}
		return(last_spot);
		}
        }
