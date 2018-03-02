/*
 * CyeBallNoiseSim.java
 */

//package EDU.gatech.cc.is.simulation;

import java.awt.*;
import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.util.Units;
import EDU.gatech.cc.is.simulation.*;
import EDU.cmu.cs.coral.util.*;

/**
 * A noisy soccerball for 16x63.
 * <B>Introduction</B><BR>
 * SoccerBallNoiseSim implements a soccer ball for RoboCup
 * soccer simulations.  The ball is also the scorekeeper and 
 * the referee; after all who would know better whether a 
 * scoring event occured?
 * <P>
 * A "shot clock" keeps track of how long since a scoring
 * event occured.  If it times-out, the ball is reset to the
 * center of the field.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class CyeBallNoiseSim extends CyeBallSim 
	{
	private final static double NOISE_MAG = 0.05;

	/**
	 * Handle a push.  This is how to kick or push the ball.
	 */
        public void push(Vec2 d, Vec2 v)
                {
                /*--- move according to the push ---*/
		  		Vec2 distance = new Vec2(d.x, d.y);
		Vec2 tmpPos = new Vec2(position);
		tmpPos.add(distance);
		boolean not_robot = true;
		    Circle2 myself = new Circle2(tmpPos, RADIUS);
                for (int i=0; i<all_objects.length; i++)
                       	{
                       	if (all_objects[i].isObstacle() &&
                               	(all_objects[i].getID() != unique_id))
                               	{
				not_robot = true;
                               	Vec2 tmp = all_objects[i].getClosestPoint(tmpPos);
						if (all_objects[i].getVisionClass() == 0) // i.e. ignores robots but not walls
							not_robot = false;
                               	if (all_objects[i].checkCollision(myself)&& not_robot)
							{

							distance = new Vec2(0.0,0.0);
							double bounce = Units.BestTurnRad(velocity.t, tmp.t);
							//velocity.sett(Math.PI + bounce + tmp.t);
							//	break;
							}
				}
			}

		position.add(distance);
		velocity = new Vec2(v.x, v.y); 
		  //	super.push(d,v);
		velocity.sett(velocity.t + (2.0*(0.5-rando.nextDouble())*NOISE_MAG));
                }
	}
