/*
 * CDTeamHetero.java 
 * 
 * by Cristian Dima starting from BasicTeam
 *
 */

import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;
//Clay not used

/**
 * Example of a simple strategy for a robot
 * soccer team without using Clay.
 * It illustrates how to use many of the sensor and
 * all of the motor methods of a SocSmall robot.
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


public class CDTeamHetero extends ControlSystemSS
{

  static final double ROBOT_RADIUS = .06;
  static final double GOAL_WIDTH = .4;

    /**
       Configure the control system.  This method is
       called once at initialization time.  You can use it
       to do whatever you like.
    */
    public void Configure()
    {
        // not used in this example.
    }
    
    
    /**
       Called every timestep to allow the control system to
       run.
    */
    public int TakeStep()
    {
        int DEBUG = 5;
        double speed=0;
        Vec2 tmp = new Vec2(0,0);

        // the eventual movement command is placed here
        Vec2    result = new Vec2(0,0);
        
        // get the current time for timestamps
        long    curr_time = abstract_robot.getTime();

        // just for debug;
        int myID = abstract_robot.getPlayerNumber(curr_time);
        
        
        /*--- Get some sensor data ---*/

        // get vector to the ball
        Vec2 myPos = abstract_robot.getPosition(curr_time);
        
        Vec2 ball = abstract_robot.getBall(curr_time);
        
        // get vector to our and their goal
        Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
        Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);

        // vector to center of the field minus one radius
        Vec2 center = new Vec2(theirgoal.x,theirgoal.y);
        center.sub(ourgoal);
        center.setr(center.r/2-1.5*ROBOT_RADIUS);
        center.add(ourgoal);

        int direction = 1; // by default, WEST team
        if (theirgoal.x < ourgoal.x)  // then EAST team
          direction = -1; 
        
        // get a list of the positions of our teammates
        Vec2[] teammates = abstract_robot.getTeammates(curr_time);
  
        // get a list of the positions of our opponents
        Vec2[] opponents = abstract_robot.getOpponents(curr_time);


        // find out if the ball is in our half or in their half
        boolean IN_OUR_HALF = true; // be pesimistic, it helps
        {
          Vec2 ball_abs = new Vec2(ball.x,ball.y);
          ball_abs.add(myPos);
          if (ball_abs.x*direction < 0)
            IN_OUR_HALF = true;
          else 
            IN_OUR_HALF = false;
          if (DEBUG==3)
            System.out.println("In ours = "+IN_OUR_HALF);
        }



        // find the closest teammate
        Vec2 closestteammate = new Vec2(99999,0);
        for (int i=0; i< teammates.length; i++)
            {
              if (teammates[i].r < closestteammate.r)
                closestteammate = teammates[i];
            }

        // find the closest opponent
        Vec2 closestopponent = new Vec2(99999,0);
        for (int i=0; i< opponents.length; i++)
            {
              if (opponents[i].r < closestteammate.r)
                closestopponent = opponents[i];
            }

        

        boolean CLOSEST_TO_BALL = true;
        for (int i=0;i<teammates.length;i++) {
          tmp = new Vec2(ball);
          tmp.sub(teammates[i]);
          if (tmp.r+ ROBOT_RADIUS < ball.r) {
            CLOSEST_TO_BALL = false;
            //   i=teammates.length;
          }
        } // end for
                      


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
        backspot.setr(abstract_robot.RADIUS*3.5);
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
        
        Vec2 awayfromopponent = new Vec2(closestopponent.x,
                                         closestopponent.y);
        awayfromopponent.sett(awayfromopponent.t+Math.PI);
            
        // Idea: decide what a robot should be doing by looking at the
        // X coord.
        
//      int ahead=0, behind=0;

//      for (int i=0; i<teammates.length;i++) {
//          if (Math.abs(teammates[i].x*direction) < 0.0001)
//              {
//                  if (DEBUG == 1) System.out.println("In line!");
//                  if (teammates[i].y > 0)
//                      ahead++;
//                  else
//                      behind++;
//              }
//          else if (teammates[i].x*direction < 0.0)
//              behind++;
//          else 
//              ahead++;

//      }
//      if (DEBUG == 1) System.out.println("MyID ="+myID+" Ahead: "+ahead+" Behind:"+behind);

        

        // THE BEHAVIOUR OF A ROBOT IN CASE IT IS THE CLOSEST TO THE
        // BALL IS THE SAME FOR ALL THE TEAMMATES. 
        if (CLOSEST_TO_BALL) {
//        if (DEBUG == 2) System.out.println("MyID ="+myID+" Ahead: "+ahead+" Behind:"+behind + "Moving to the kickspot");

          if (ball.r > .1) {
            speed = 1;
            result = kickspot;
          }
          else {
            speed = .5;
            if (kickspot.r > ball.r) // trying to avoid scoring in my own goal :)
              {
                Vec2 false_kickspot = new Vec2(0,0);
                false_kickspot.sety(kickspot.y + 
                  ((myPos.y > ourgoal.y) ? (-1):(1))*(2)*ROBOT_RADIUS);
                result = false_kickspot;
              }
            else result = kickspot;}
        } else {
       
          // HERE IS WHAT TO DO WHEN YOU DON'T HAVE THE BALL
          // I should try to score
          if (myID == 4) {
            if (DEBUG == 1) System.out.println("I should score");
            if (IN_OUR_HALF)
              {
                result=center;
                if (result.r > 0.5*ROBOT_RADIUS)
                  speed=1;
                else
                  speed=0;
              }
            else
              {
                result = kickspot;
                result.sety(result.y - 3*ROBOT_RADIUS);
                speed = 1;
              }
          } // endif ahead=0
       
          
          // I should help the forward score
          if (myID == 3) {
            if (DEBUG == 1) System.out.println("I should help the forward score");
            if (!IN_OUR_HALF) {
//            Vec2 forward=new Vec2(0,0); // this is the first attacker, the one I'm
//            // supposed to help
//            for (int i=0;i<teammates.length;i++) 
//              if (teammates[i].x*direction > 0) {
//                forward = new Vec2(teammates[i].x,teammates[i].y);
//                i = teammates.length;
//              } // endif
//            result = forward;
//            result.sety(2*ball.y-forward.y);
//            if (forward.r < 3*ROBOT_RADIUS) {
//              if (forward.y > ball.y)
//                result.sety(forward.y-6*ROBOT_RADIUS);
//              else
//                result.sety(forward.y+6*ROBOT_RADIUS);
//              }
//            speed = 1;
              result = backspot;
              speed = 1;
            }
            else { // if in our half
              result = center;
              result.sety(result.y - 6*ROBOT_RADIUS);
              if (result.r > 0.5*ROBOT_RADIUS)
                  speed=1;
                else
                  speed=0;
            }
          }
          
          // I should block the attacker
          if (myID == 2) {
            if (DEBUG == 1) System.out.println("I should block the attacker");
                    if (!CLOSEST_TO_BALL) { // I'll place myself in the way of
              // the ball, but rather far from
              // the goalie. (avoid blocking)
              Vec2 virtual_center = new Vec2(ourgoal.x,ourgoal.y);
              virtual_center.setx(virtual_center.x - direction * GOAL_WIDTH);
              result = new Vec2(ball);
              result.sub(virtual_center);
              result.setr(3*GOAL_WIDTH);
              result.add(virtual_center);
              if (result.r > .5*ROBOT_RADIUS)
                speed = 1;
              else
                speed = 0;
            }
          }
          
          // I'd better get the ball fast!!
          if (myID == 1) {
            if (DEBUG == 1) System.out.println("I'd better get the ball fast!!");
            if (!CLOSEST_TO_BALL) { // I'll place myself in the way of
              // the ball, but rather far from
              // the goalie. (avoid blocking)
              Vec2 virtual_center = new Vec2(ourgoal.x,ourgoal.y);
              virtual_center.setx(virtual_center.x - direction * GOAL_WIDTH);
              result = new Vec2(ball);
              result.sub(virtual_center);
              result.setr(2*GOAL_WIDTH);
              result.add(virtual_center);
              if (result.r > .5*ROBOT_RADIUS)
                speed = 1;
              else
                speed = 0;
            }
          }
          
          // I am the goalie
          if (myID == 0) {
            if (DEBUG == 1) System.out.println("I am the goalie");
            if (!CLOSEST_TO_BALL) { // I'll place myself in the way of
              // the ball, in the goal.
              Vec2 virtual_center = new Vec2(ourgoal.x,ourgoal.y);
              double coeff=0;
              virtual_center.setx(virtual_center.x - direction * GOAL_WIDTH);
              result = new Vec2(ball);
              result.sub(virtual_center);
              if (DEBUG==4)
                System.out.println(closestopponent.r);
              if (closestopponent.r <=2*ROBOT_RADIUS)
                // then, I am blocked by some stupid robot
                coeff = 1.7;
              else
                coeff = 1.1;
              result.setr(coeff*GOAL_WIDTH);
              result.add(virtual_center);
              if (result.r > .5*ROBOT_RADIUS)
                speed = 1;
              else
                speed = 0;
            } // endif !CLOSEST_To_BALL
            
          }
        }
        
        if (closestteammate.r <= .10)
          result = awayfromclosest;
        
        // THIS AVOIDS GETTING STUCK, BUT IT FAVORS THE OTHER TEAM BY
        // LETTING IT ADVANCE

                if ((closestopponent.r <= .10) && (myID != 0))  
          result = awayfromopponent;

        
        /*--- Send commands to actuators ---*/
        // set the heading
        
        abstract_robot.setSteerHeading(curr_time, result.t);
          
        // set speed at maximum
        abstract_robot.setSpeed(curr_time, speed);
        
        
        // kick it if we can
        if (abstract_robot.canKick(curr_time)) 
          abstract_robot.kick(curr_time);
        
        
        // tell the parent we're OK
        return(CSSTAT_OK);
    }
} // end class CDTeamHetero


