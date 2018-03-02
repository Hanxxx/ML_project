/*--------------------------------------------------------------------*/
/*
 * LoneForwardTeamHomoG.java
 */
import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;
import java.lang.*;

/**
 * @author Bryan Nagy
 * @version $Revision: 1.1 $
 *
 *      Only faintly derived from the BasicTeam code for things like
 *      finding the closest teammate and calculating the kickspot.
 *  All actual control and decision algorithims are unique,
 */

public class LoneForwardTeamHomoG extends ControlSystemSS
        {
        /**
        Configure the control system.  This method is
        called once at initialization time.  You can use it
        to do whatever you like.
        */
        public void Configure()
                {
                // not used
                }
                
        
        /**
        Called every timestep to allow the control system to
        run.
        */
        public int TakeStep()
                {
                // the eventual movement command is placed here
                Vec2    result = new Vec2(0,0);
                // get the current time for timestamps
                long    curr_time = abstract_robot.getTime();

                //--- Get some sensor data ---
                // get vector to the ball
                Vec2 ball = abstract_robot.getBall(curr_time);

                // get vector to our and their goal
                Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
                Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);
                // get a list of the positions of our teammates
                Vec2[] teammates = abstract_robot.getTeammates(curr_time);
                // find the closest teammate
                Vec2 closestteammate = new Vec2(99999,0);
                for (int i=0; i< teammates.length; i++) { 
                        if (teammates[i].r < closestteammate.r)
                                closestteammate = teammates[i];
                }
                
                // find if we are the robot on our team closest to the ball
                double distance=0;
                double minDistance=99999;

                for (int i=0; i< teammates.length; i++){
                       
distance=Math.pow((Math.pow((ball.x-teammates[i].x),2.0)+Math.pow((ball.y-teammates[i].y),2.0)),.5);
                        if (distance < minDistance){
                                minDistance = distance;
                        }
                }

                boolean iAmClosest=true;
                if (ball.r*.95 > minDistance){
                                iAmClosest =false;
                }
        
                //--- now compute some strategic places to go ---
                // compute a point one robot radius
                // behind the ball.
                Vec2 kickspot = new Vec2(ball.x, ball.y);
                kickspot.sub(theirgoal);
                kickspot.setr(abstract_robot.RADIUS);
                kickspot.add(ball);
                
                //we will use this to test if we are in front of or behind the ball
                Vec2 frontspot = new Vec2(ball.x, ball.y);
                frontspot.sub(theirgoal);
                frontspot.setr(-abstract_robot.RADIUS*2);
                frontspot.add(ball);
                // compute a north and south spot
                Vec2 northspot = new
Vec2(ball.x,ball.y+abstract_robot.RADIUS*2);
                Vec2 southspot = new
Vec2(ball.x,ball.y-abstract_robot.RADIUS*2);
                // compute a position between the ball and defended goal
                Vec2 goaliepos = new Vec2(ourgoal.x + ball.x,
                                ourgoal.y + ball.y);
                goaliepos.setr(goaliepos.r*0.5);
                // a direction away from the closest teammate.
                Vec2 awayfromclosest = new Vec2(closestteammate.x,
                                closestteammate.y);
                awayfromclosest.sett(awayfromclosest.t + Math.PI);
                //the LoneForwardTeamPlan:

                //if I am the closest member of my team to the ball, 
                if(iAmClosest){
                        //then it will be my job to put it in
                        //If I am between the ball and their goal, 
                        //that is, on the wrong side of the ball,
                        if(kickspot.r>frontspot.r){
                                //I will first 'swirl' around the ball,
                                //if I am above the ball, 
                                if(ball.y<0){
                                        result=northspot;
                                       
abstract_robot.setDisplayString("Swirling North");
                                //I will swirl north 
                                }else{
                                       
abstract_robot.setDisplayString("Swirling South");
                                        result=southspot;
                                        //else I will swirl south
                                }
                        }
                        else{
                        //otherwise I will head for the kickspot behind the ball.
                                //if I am already at the kickspot
                                if(kickspot.r<abstract_robot.RADIUS*.1){
                                                //I will continue toward it
                                                result = ball;
                                          //and if I can kick the ball
                                                //and I am close enough to score...
                                                if
(abstract_robot.canKick(curr_time) && theirgoal.r<=1){
                                                  //I will kick the ball
                                                       
abstract_robot.kick(curr_time);
                                                }
                                                //and continue toward it
                                                result = ball;
                                }else{

abstract_robot.setDisplayString("Seeking Kickspot");
                                        result=kickspot;
                                }
                        }
                }else{
                        //but if I am not the closet member of my team to the ball, 
                        //I will go defend the goal.
                        //by driving towards the center of the goal
                        abstract_robot.setDisplayString("Defending");

                        result=ourgoal;
                        result.sety((ourgoal.y+goaliepos.y*0.5)/2);
                        result.setx(ourgoal.x+ball.x*0.01);
                        
                        //but if I am within a certain distance of the x value of the goal, 
                        // and within a certain distance of my teammates,
                        if((Math.abs(ourgoal.x)<2)&&(closestteammate.r<0.08)){
                                // I will move away from the closest one.
                                //but only in the y vector. 
                                result.sety(awayfromclosest.y*1.5);
                                result.setx(0);
                                abstract_robot.setDisplayString("Defending and avoiding Teammate");
                        }
                        
                        //detect if we are being trapped...
                        if((Math.abs(ourgoal.x)<abstract_robot.RADIUS*5)
&&(Math.abs((ourgoal.y+goaliepos.y*0.5)/2)>abstract_robot.RADIUS*3)){
                        //and if so, circle around the trapper...
                                result.setx(ourgoal.x*-500);
                                result.setx(ourgoal.y*2);
                                abstract_robot.setDisplayString("Defending and avoiding Being Trapped");
                        }
                }

                //--- Send commands to actuators ---
                // set the heading
                abstract_robot.setSteerHeading(curr_time, result.t);
                // set speed at maximum
                abstract_robot.setSpeed(curr_time, 1.0);
                // tell the parent we're OK
                return(CSSTAT_OK);
                }
        }



