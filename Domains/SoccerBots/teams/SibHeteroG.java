/*
 * SibHeteroG.java
 */

import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;
//Clay not used

/**
 * Mark Sibenac
 * 1999-10-05
 *
 * Sib's Heterogeneous Team derived a lot from the BasicTeam by Tucker Balch
 *
 * Major changes include a cherry picker and a goalie who hugs the goal.
 *
 * If the player is pushing the ball towards his own goal, he is forbidden to
 * kick. Only the goalie is allowed to kick when the ball is close to our goal.
 *
 * The cherry picker waits in the middle of the field for the ball to appear
 * after a reset. It is taking advantage of the system, and it works!
 *
 */


public class SibHeteroG extends ControlSystemSS
        {
        public void Configure()
                {
                }
                
        public int TakeStep()
                {
                // the eventual movement command is placed here
                Vec2    result = new Vec2(0,0);

                Vec2    mypos = new Vec2(abstract_robot.getPosition(-1));

                // get the current time for timestamps
                long    curr_time = abstract_robot.getTime();


                /*--- Get some sensor data ---*/
                // get vector to the ball
                Vec2 ball = abstract_robot.getBall(curr_time);

                // get vector to our and their goal
                Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
                Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);

                // get a list of the positions of our teammates
                Vec2[] teammates = abstract_robot.getTeammates(curr_time);

                // find the closest teammate
                Vec2 closestteammate = new Vec2(99999,0);
                for (int i=0; i< teammates.length; i++)
                        {
                        if (teammates[i].r < closestteammate.r)
                                closestteammate = teammates[i];
                        }


                /*--- now compute some strategic places to go ---*/
                // compute a point one robot radius
                // behind the ball towards the goal
                Vec2 kickspot = new Vec2(ball.x, ball.y);
                kickspot.sub(theirgoal);
                kickspot.setr(abstract_robot.RADIUS);
                kickspot.add(ball);

                // compute a point three robot radii
                // behind the ball.
                Vec2 backspot = new Vec2(ball.x, ball.y);
                backspot.sub(theirgoal);
                backspot.setr(abstract_robot.RADIUS*5);
                backspot.add(ball);

                // compute the cherry picker's spot
                Vec2 cherry = new Vec2(-0.2,0.0);
                if (mypos.x+ourgoal.x > 0)
                    cherry.setx(0.2);
                cherry.sub(mypos);

                // compute a north and south spot
                Vec2 northspot = new Vec2(backspot.x,backspot.y+1.525/4);
                Vec2 southspot = new Vec2(backspot.x,backspot.y-1.525/4);

                // compute a position between the ball and defended goal
                double y = ball.y;
                if (ball.y > 0.0 && mypos.y > 0.2) y = 0.0;
                else if (ball.y < 0.0 && mypos.y < -0.2) y = 0.0;
                Vec2 goaliepos = new Vec2(ourgoal.x, y);

                // a direction away from the closest teammate.
                Vec2 awayfromclosest = new Vec2(closestteammate.x,
                                closestteammate.y);
                awayfromclosest.sett(awayfromclosest.t + Math.PI);


                /*--- go to one of the places depending on player num ---*/
                int mynum = abstract_robot.getPlayerNumber(curr_time);
                
                double far_away = abstract_robot.RADIUS * 5;
                double kick_dist = abstract_robot.RADIUS*1.1;

                /*--- Goalie ---*/
                if (mynum == 0)
                    {
                        // go to the goalie position if far from the ball
                        if (ball.r > 0.5)
                            {
                                result = goaliepos;
                            } // otherwise go to kick it
                        else if (ball.r > kick_dist) 
                            result = kickspot;
                        else 
                            result = ball;
                    }

                /*--- midback ---*/
                else if (mynum == 1)
                    {
                        // go to a midback position if far from the ball
                        if ((ball.r > far_away) || (mypos.r > far_away))
                            result = cherry; // cherry picking position
                        // otherwise go to kick it
                        else if (ball.r > kick_dist)
                            result = kickspot;
                        else 
                            result = ball;
                        // keep away from others
                        if (closestteammate.r < abstract_robot.RADIUS*2)
                            {
                                result = awayfromclosest;
                            }
                    }

                else if (mynum == 2)
                        {
                        // go to a the northspot position if far from the ball
                            if (ball.r > far_away)
                                result = northspot;
                        // otherwise go to kick it
                            else if (ball.r > kick_dist)
                                result = kickspot;
                            else 
                                result = ball;
                        // keep away from others
                        if (closestteammate.r < abstract_robot.RADIUS*2)
                                {
                                result = awayfromclosest;
                                }
                        }

                else if (mynum == 4)
                        {
                        // go to a the southspot position if far from the ball
                            if (ball.r > far_away)
                                result = southspot;
                        // otherwise go to kick it
                            else if (ball.r > kick_dist)
                                result = kickspot;
                            else 
                                result = ball;
                        // keep away from others
                        if (closestteammate.r < abstract_robot.RADIUS*2)
                                {
                                result = awayfromclosest;
                                }
                        }

                /*---Lead Forward ---*/
                else if (mynum == 3)
                        {
                        // if we are more than 4cm away from the ball
                            if (ball.r > kick_dist)
                                // go to a good kicking position
                                result = kickspot;
                        else
                                // go to the ball
                                result = ball;
                        }


                /*--- Send commands to actuators ---*/
                // set the heading
                abstract_robot.setSteerHeading(curr_time, result.t);

                // set speed at maximum
                abstract_robot.setSpeed(curr_time, 1.0);

                // kick it if we can only if we are not going to kick it
                // into the goal
                if (abstract_robot.canKick(curr_time))
                    {
                        if ((ourgoal.x+mypos.x < 0 && mypos.x > -2.74/4) ||
                            (ourgoal.x+mypos.x > 0 && mypos.x < 2.74/4) ||
                            (mynum == 0))
                            abstract_robot.kick(curr_time);
                    }
                // tell the parent we're OK
                return(CSSTAT_OK);
                }
        }
