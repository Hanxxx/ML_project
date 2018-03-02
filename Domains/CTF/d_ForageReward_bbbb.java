/*
 * d_ForageReward_bbbb.java
 */

import java.lang.*;
import EDU.gatech.cc.is.abstractrobot.GripperActuator;
import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.clay.*;

/**
 * This is the robot's reward function.  1 if just dropped an article
 * near the designated homebase.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */


public class d_ForageReward_bbbb extends NodeDouble
	{
	/** 
	 * Turn debug printing on or off.
	 */
	public static final boolean DEBUG = Node.DEBUG;
	private GripperActuator	abstract_robot;
	private	NodeBoolean[] embedded;
	private boolean last_holding_r = false;
	private boolean last_close_r = false;
	private boolean last_holding_b = false;
	private boolean last_close_b = false;

	/**
	 * Instantiate a d_ForageReward_bbbb schema.
	 * @param cr NodeBoolean, indicates if the robot
         *               is close enough to the red base to drop the thing.
	 * @param hr NodeBoolean, indicates if the robot
         *               is holding a red object or not.
	 * @param cb NodeBoolean, indicates if the robot
         *               is close enough to the blue base to drop the thing.
	 * @param hb NodeBoolean, indicates if the robot
         *               is holding a blue object or not.
	 */
	public d_ForageReward_bbbb(NodeBoolean cr, NodeBoolean hr,
		NodeBoolean cb, NodeBoolean hb)
		{
		if (DEBUG) System.out.println("d_ForageReward_bbbb: instantiated");
		embedded = new NodeBoolean[4];
		embedded[0] = cr;
		embedded[1] = hr;
		embedded[2] = cb;
		embedded[3] = hb;
		}

        double  last_val = -1;
        long    lasttime = 0;
	/**
	 * Return a double representing reward.
	 * @param timestamp long, only get new information 
	 * 		if timestamp > than last call
         *		or timestamp == -1.
	 * @return the type of the object, 0 if empty.
	 */
	public double Value(long timestamp)
		{
		if (DEBUG) System.out.println("d_ForageReward_bbbb: Value()");

                if ((timestamp > lasttime)||(timestamp == -1))
                        {
                        /*--- reset the timestamp ---*/
                        if (timestamp > 0) lasttime = timestamp;

			/*--- get the info ---*/
			boolean close =   embedded[0].Value(timestamp);
			boolean holding = embedded[1].Value(timestamp);

			/*--- evaluate ---*/
			if ((last_holding_r==true)&&(holding==false)
				&&((last_close_r==true)||(close==true)))
				last_val = 1;
			else
				last_val = -1;
			
			//System.out.println("red " + close + holding +
				//last_close_r + last_holding_r);

			/*--- save ---*/
			last_close_r = close;
			last_holding_r = holding;

			/*--- get the info for blue ---*/
			close =   embedded[2].Value(timestamp);
			holding = embedded[3].Value(timestamp);

			/*--- evaluate ---*/
			if ((last_holding_b==true)&&(holding==false)
				&&((last_close_b==true)||(close==true)))
				last_val = 1;

			//System.out.println("blue " + close + holding +
				//last_close_b + last_holding_b + last_val);

			last_close_b = close;
			last_holding_b = holding;
			}

		return(last_val);
		}
        }
