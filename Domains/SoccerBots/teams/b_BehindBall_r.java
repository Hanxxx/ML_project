/*
 * b_BehindBall_r.java
 */

import java.lang.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.util.Vec2;

/**
 * Report if behind the ball or not.
 * This module is a node used in Clay to configure soccer behaviors.
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


public class b_BehindBall_r extends NodeBoolean
	{
	/** 
	 * Turn debug printing on or off.
	 */
	public static final boolean DEBUG = Node.DEBUG;
	private SocSmall abstract_robot;

	/**
	 * Instantiate a b_BehindBall_r schema.
	 * @param ar SocSmall, the abstract_robot object 
	 * that provides hardware support.
	 */
	public b_BehindBall_r(SocSmall ar)
		{
		if (DEBUG) System.out.println("b_BehindBall_r: instantiated");
		abstract_robot = ar;
		}

	long	last_t = 0;
	boolean	last_val = false;
	/**
	 * Return a boolean indicating if behind the ball or not.
	 * @param timestamp long, only get new information 
	 *        if timestamp > than last call or timestamp == -1.
	 * @return true if behind the ball, false otherwise.
	 */
	public boolean Value(long timestamp)
		{
		if (DEBUG) System.out.println("b_BehindBall_r: Value()");

		if ((timestamp > last_t) || (timestamp == -1))
			{
			if (timestamp != -1) last_t = timestamp;

			Vec2 ball = abstract_robot.getBall(timestamp);
			Vec2 goal = abstract_robot.getOpponentsGoal(timestamp);

			last_val = false;
			// check for same sign
			if (((ball.x<0)&&(goal.x<0))||((ball.x>0)&&(goal.x>0)))
				if (Math.abs(ball.x) >= 
					(abstract_robot.RADIUS/2))
					last_val = true;
			}
		return(last_val);
		}
        }
