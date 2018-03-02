/*
 * v_SwirlToBall_v.java
 */

import	EDU.gatech.cc.is.util.*;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import  EDU.cmu.cs.coral.abstractrobot.*;

/**
 * Generates a vector that swirls the robot behind the ball so that
 * it pushes the ball towards the goal location
 *
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997, 1998 Tucker Balch
 *
 * @author Rosemary Emery
 * @version $Revision: 1.4 $
 */


// NOTE: THIS CLASS ASSUMES BALL IS OF RADIUS 0.2 - it is of radius 0.1 in real life and should be an input

public class v_SwirlToBall_v  extends NodeVec2
	{
	  public final static boolean DEBUG = Node.DEBUG; //true;
	private NodeVec2	steering_configuration;
	private NodeInt 	state_monitor;

	    protected double stopDistance;

	NodeVec2 PS_GLOBAL_POS;
	NodeVec2 PS_CLOSEST0;
	NodeBoolean PF_TARGET0_VISIBLE;
	NodeVec2 PS_HOMEBASE0;
	NodeVec2 PS_GLOBAL_HEADING;
	b_ResetableBoolean_ PF_BALL_TRACKED_OK,PF_WAITING_TO_KICK_BALL,PF_DONE_KICKING_BALL;
        boolean BALL_CLOSE, IN_ANGLE_CONE;  
	Vec2 globalBall;

	double CAMERA_DISTANCE = SimpleCye.VISION_RANGE;
        double FIELD_OF_VIEW = 0.8*SimpleCye.VISION_FOV_RAD;
	double veryCloseToBall = 0.4;
        double thetaMax = 3.0*Math.PI/8.0;
	double innerMaxDistance = 0.7;
	double outerMaxDistance = 1.0;
	    
	public v_SwirlToBall_v(NodeVec2 GlobalPosRobot, NodeVec2 ClosestTarget, NodeVec2 HB, NodeBoolean TV,double stopDist,b_ResetableBoolean_ ballTrackedOk,b_ResetableBoolean_ WaitingToKickBall,b_ResetableBoolean_ DoneKickingBall, NodeVec2 robotHeading)

		{
		if (DEBUG) System.out.println("v_SwirlToBall__v: instantiated.");
		PS_GLOBAL_POS = GlobalPosRobot;
		PS_CLOSEST0 = ClosestTarget;
		PS_HOMEBASE0 = HB;
		PF_TARGET0_VISIBLE = TV;
		globalBall = new Vec2(0.0,0.0);
		stopDistance=stopDist;
		PF_BALL_TRACKED_OK=ballTrackedOk;
		PF_WAITING_TO_KICK_BALL=WaitingToKickBall;
		PF_DONE_KICKING_BALL=DoneKickingBall;
		IN_ANGLE_CONE = false;
		BALL_CLOSE = false;
		PS_GLOBAL_HEADING = robotHeading;
	}


	long	lasttime = 0;
	public Vec2 Value(long timestamp)
	    {
	      Vec2	result = new Vec2(0.0, 0.0);
	      double	dresult;
	      Vec2	p;
	      double alpha;
	      double speed;
	      Vec2 closestBall;

	      PF_DONE_KICKING_BALL.setValue(false);
	      PF_WAITING_TO_KICK_BALL.setValue(false);
	      PF_BALL_TRACKED_OK.setValue(true);

	      if ((timestamp > lasttime)||(timestamp == -1))
		  {
		      if (DEBUG) System.out.println("v_LinearAttraction_v:");
	       	/*--- reset the timestamp ---*/
       		if (timestamp > 0) lasttime = timestamp;

		Vec2 robotPosition = PS_GLOBAL_POS.Value(timestamp); // global position

		if (PF_TARGET0_VISIBLE.Value(timestamp))
		  {
		  closestBall = PS_CLOSEST0.Value(timestamp); // with respect to the robot
		  closestBall.setr(closestBall.r); // so it ball radius closer to robot 
		  globalBall = new Vec2(robotPosition);
		  globalBall.add(closestBall);
		  }
		else // if ball isn't visible
		  {
		    closestBall = new Vec2(globalBall);
		    closestBall.sub(robotPosition);

		    //if ball should be visible
		    Vec2 robotHeading = PS_GLOBAL_HEADING.Value(timestamp);
		    double angleToBall = robotHeading.t - closestBall.t;
		    
		    if (closestBall.r <= CAMERA_DISTANCE && Math.abs(angleToBall) <= 0.5*FIELD_OF_VIEW) {
			PF_BALL_TRACKED_OK.setValue(false);
                        BALL_CLOSE = false;
			IN_ANGLE_CONE = false;

		        return result;
		    }
		  }

		Vec2 goalLocation = PS_HOMEBASE0.Value(timestamp); // right hand goal location
		
		//		goalLocation.sub(robotPosition); // goal location with respect to robot
		Vec2 ballToGoal = new Vec2(goalLocation);
		ballToGoal.sub(closestBall);
		

		if (closestBall.r < 0.27+stopDistance && ballInCone(closestBall,goalLocation )) 
		  {
		      if (stopDistance<=0)
			  {
			      PF_DONE_KICKING_BALL.setValue(true);
			  } else {
			      PF_WAITING_TO_KICK_BALL.setValue(true);
			  }
  
		    System.out.println("at ball, waiting for next state to move ball");
                    BALL_CLOSE = false;
                    IN_ANGLE_CONE = false;
		    return result;
		  }

        	double theta;
	   
		if (goalLocation.t ==  ballToGoal.t)
		  {
		    theta = 0.0;
		  }
		else
		  {
		    double phi = ((Math.pow(goalLocation.r,2) - Math.pow(ballToGoal.r,2) - 
					Math.pow(closestBall.r,2))/
				       (-2.0*ballToGoal.r*closestBall.r));
		    phi = Math.acos(phi);
		    theta = Math.abs(Math.PI-phi);
		  }

		// setting flags
		if ((theta <= thetaMax) || (IN_ANGLE_CONE && (closestBall.r<=veryCloseToBall)))
		{
		    IN_ANGLE_CONE = true;
                }
                if (closestBall.r < innerMaxDistance)
		{
		    BALL_CLOSE = true;
                }
		else if (closestBall.r > outerMaxDistance)
	        {
		    BALL_CLOSE = false;
                }
		double deltaXBall, deltaYBall;
		if (closestBall.r != 0.0)
		  {
		  deltaXBall = closestBall.x/closestBall.r;
		  deltaYBall = closestBall.y/closestBall.r;
		  }
		else
		  {
		  deltaXBall = 0.0;
		  deltaYBall = 0.0;
		  }

		// we want the robot to go to the ball, not push it forward - that will be handled by
		// a separate motor schema
		
		double deltaXPerp, deltaYPerp;
		
		// 4 cases - still something wrong with them? - top ones goes, if closestBall behind robot
		// still not quite right
		double turnDirection = 0.0;
		if (ballToGoal.x >= 0.0)
		    {
			if (aboveLine(robotPosition, ballToGoal, globalBall))
			    turnDirection = -1.0; // CCW
			else
			    turnDirection = 1.0; // CW
		    }
		else
		    {
			if (aboveLine(robotPosition, ballToGoal, globalBall))
			    turnDirection = 1.0; // CW
			else
			    turnDirection = -1.0; // CCW
		    }
       
	       deltaXPerp = -1.0*turnDirection*deltaYBall;
       	       deltaYPerp = turnDirection*deltaXBall;

	       System.out.println("robot to ball " + " " + turnDirection + " " +deltaXBall + " " + deltaYBall);
	       System.out.println("perp " + deltaXPerp + " " + deltaYPerp);
		
		if (IN_ANGLE_CONE)
		  {
		    // alpha should perhaps also depend on distance from ball
		   
		    alpha = theta/thetaMax;
		    result = new Vec2(alpha*deltaXPerp + (1.0-alpha)*deltaXBall,
				      alpha*deltaYPerp + (1.0-alpha)*deltaYBall);
		    System.err.println("in cone");
		  }
		else if (BALL_CLOSE)

		  {
		   
		    result = new Vec2(deltaXPerp, deltaYPerp);
		    System.err.println("perp only");
		  }
		else
		  {
			result = new Vec2(closestBall.x - 0.75*ballToGoal.x/ballToGoal.r, 
					  closestBall.y-0.75*ballToGoal.y/ballToGoal.r); // was 0.5
			System.err.println("old bee line");
	      
		  }
		// STEER
		//	System.out.println("result " + result.t + " " + result.r);
		}

	      return result;
	      }

	  private boolean signSame(double x,double y)
               {
		 if ((x >=0.0 && y >= 0.0) || (x <=0.0 && y <=0.0)) // same sign within some epsilong
		   return true;
		 else
		   return false;
	       }
	  private boolean aboveLine(Vec2 point, Vec2 line, Vec2 startingPoint)
               {
		 double m;
		 if (line.x !=0)
		   m = line.y/line.x;
		 else
		   {
		     if (point.x <= startingPoint.x)
		       return true;
		     else
		       return false;
		   }
		 double b = startingPoint.y - m*startingPoint.x;
		 if (point.y >= (m*point.x + b) )
		   return true;
		 else
		   return false;
	       }

	  private boolean ballInCone(Vec2 ball, Vec2 goal)
	      {
		  double theta = Math.PI/6; // 30 degrees

		  if (Math.abs(ball.t) < theta && Math.abs(goal.t)< theta)
		      return true;
		  else 
		      return false;
	      }
	      
}





