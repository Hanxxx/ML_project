/*
 * Kechze.java
 *
 * KEnt, CHris, and ZEllyn's JavaSoccer Team
 * kent@cc, jurney@cc, zellyn@cc
 * cs4324 - Spring 1998
 * Chris Atkeson's Class
 *
 * Assignment One
 *
 *
 *
 * Based on BrianTeam.java by Brian McNamara, although the code itself was
 *  not cut and pasted - it was written from scratch.
 *
 */

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.abstractrobot.*;
import java.io.*;

public class Kechze extends ControlSystemSS
{
  // The current time
  long curr_time;

  // Position of ball, vector to ball, vectors to goals, position of robot
  Vec2 ball, EgoBall, ourgoal, theirgoal, me;

  // Arrays of team-members
  Vec2 Teammates[], Opponents[];

  // Useful constants
  static final double PI = Math.PI, PI2 = 2*Math.PI;
  static final double GOAL_WIDTH = .4, BALL_RADIUS = .02;
  // goal a little smaller to be safe
  static final double ROBOT_RADIUS = .06;

  // By default, robots move at full speed
  static final double DEFAULT_SPEED = 1.0;

  // How close robots must get to a target spot before stopping
  static final double CLOSE_TO_SPOT = 0.01;

  // How close ball must be to goal for goalie to act "aggressively"
  static final double CLOSE_TO_GOAL = 0.4;

  // How many milliseconds ahead the goalie predicts the ball's position
  static final long GOALIE_LOOK_AHEAD = (long)200.0;

  // How many milliseconds ahead players predict the ball's position when
  //  simply running in the direction of the ball (bottom-out default
  //  behavior)
  static final long DEFAULT_LOOK_AHEAD = (long)600.0;

 // How far back from the center robots wait for the ball to re-spot
  static final double CENTER_OFFSET = 0.1;

  // How far the ball must be from the center for respotting-wait to happen
  static final double CENTER_RADIUS = 1.3;

  long last_time; // what the time was last step
  Vec2 last_ball; // where the ball was last step

  long steps;
  double ratio;

 /*
 Configure the control system.
 */
  public void Configure()
  {
    last_time = (long)0.0; //zjh
    last_ball = new Vec2(0.0,0.0); //zjh

    steps = 0;
    ratio = 0.0;
  }

  /**
 Called every timestep to allow the control system to run.
 */
  public int TakeStep()
  {
    // the eventual movement command is placed here
    Vec2 result = new Vec2(0,DEFAULT_SPEED);

    // get the current time for timestamps
    curr_time = abstract_robot.getTime();

    me = abstract_robot.getPosition(curr_time);

    boolean KickBall = false;

    /*--- Get some sensor data ---*/
    // get vector to the ball
    EgoBall = abstract_robot.getBall(curr_time);
    ball = new Vec2(EgoBall.x, EgoBall.y);
    ball.add(me);

    // get vector to our and their goal
    ourgoal = abstract_robot.getOurGoal(curr_time);
    theirgoal = abstract_robot.getOpponentsGoal(curr_time);

    // get a list of the positions of our teammates
    Teammates = abstract_robot.getTeammates(curr_time);
    Opponents = abstract_robot.getOpponents(curr_time);

    if(HaveBall(me))
      {
        abstract_robot.setDisplayString("Have Ball");
        if (ClearShot(me) && theirgoal.r < .35)
          {
            //System.out.print("*ClearShot  ");
            abstract_robot.setDisplayString("Shooting");
            KickBall = true;
            //    result.sett(TopGoalPost().t); //theirgoal;
            result.sett(EgoBall.t); //theirgoal;
          }
        else if(ClosestToMyGoal(me))
          {
            //System.out.print("*Closest to goal ");
            abstract_robot.setDisplayString("Clearing");
            KickBall = true;
            result.sett(EgoBall.t);
          }
        else
          {
            //System.out.print(abstract_robot.getPlayerNumber(curr_time));
            //System.out.print("*Closest to ball  ");
            result.sett(GoBehindBall());
            //System.out.println(result.t + "  ");
            //System.out.println("Else* " + result.t);
          }
      }
    else
      {
        //System.out.print("*Dont have ball  ");
        if (ClosestToBall(me))
          {
            //System.out.print(abstract_robot.getPlayerNumber(curr_time));
            //System.out.print("*Closest to ball  ");
            abstract_robot.setDisplayString("Going to Ball");
            result.sett(GoBehindBall());
            //System.out.println(result.t + "  ");
          }
        //-dedicated goalie now  else if(ClosestToMyGoal(me)) {result =
	//                                                     goalieFunc();}
        else

          if (ClosestToCenter(me) && (ball.r>CENTER_RADIUS))
            {
              Vec2 ourwait;
              abstract_robot.setDisplayString("Center");
              if (ourgoal.x > 0)
                {
                  ourwait = new Vec2(CENTER_OFFSET,0);
                }
              else
                {
                  ourwait = new Vec2(-CENTER_OFFSET,0);
                }

              ourwait.sub(me);
              result = easeToRelativeSpot(ourwait, (EastTeam(me)?PI:0.0));

            }

          else {
            //System.out.print("*else  ");
            //PUT code here somewhere
            result = ClosestOpponent();
            if( result.r < ROBOT_RADIUS * 2)
              {
                abstract_robot.setDisplayString("Avoiding Opp.");
                result.sett(NormalizePI(result.t + PI));
              }
            else
              {
                result = ClosestTeammate();
                if( result.r < ROBOT_RADIUS * 4)
                  {
                    abstract_robot.setDisplayString("Avoiding Teammate");
                    result.sett(NormalizePI(result.t + PI));
                  }
                else
                  {
                    abstract_robot.setDisplayString("Find Ball");
                    result = predictEgoBall(curr_time+DEFAULT_LOOK_AHEAD);
//zjh
                  }
              }
            result.setr(DEFAULT_SPEED);
          }
      }
    //System.out.println("");System.out.println("");
    if (abstract_robot.getPlayerNumber(curr_time)==0)
      {
        abstract_robot.setDisplayString("Goalie");
        result = goalieFunc();
        //    System.out.println("Goalie: " + result);
      }

    steps++;
    if (steps>100) steps=100;
    ratio = ratio * (1.0-1.0/steps) + 1.0/steps * (haveGoalie()?1.0:0.0);
    //  System.out.println(ratio);
    if ((abstract_robot.getPlayerNumber(curr_time)==4) && (ratio>0.7))
      {
        abstract_robot.setDisplayString("Block Goalie");
        result = blockFunc();
      }

    /*--- Send commands to actuators ---*/
    // set the heading
    abstract_robot.setSteerHeading(curr_time, result.t);
    // set speed at maximum
    abstract_robot.setSpeed(curr_time, result.r);

  // kick it if we can & desired
    if (abstract_robot.canKick(curr_time) && KickBall)
      abstract_robot.kick(curr_time);

    last_time = curr_time;  //zjh
    last_ball = new Vec2(ball.x, ball.y); //zjh

    // tell the parent we're OK
    return(CSSTAT_OK);
  }

  /* GoBehindBall
   *  Get behind the ball, and try to turn it towards their goal
   *  Described more fully in the html documentation
   */
  double GoBehindBall()
       // which theta to goto to go behind the ball (me->ball->goal)
  {
    Vec2 result;
    result=EgoBall;
    //  if(CloseToBall(me) && !HaveBall(me))
    {
      Vec2 kickspot = new Vec2(EgoBall.x, EgoBall.y);
      kickspot.sub(theirgoal);
      kickspot.sety(kickspot.y+ROBOT_RADIUS*2);
      //   kickspot.sub(BottomGoalPost());
      Vec2 B = new Vec2(kickspot.x, kickspot.y);
      Vec2 C = new Vec2(kickspot.x, kickspot.y);
      Vec2 D = new Vec2(kickspot.x, kickspot.y);
      kickspot.setr(ROBOT_RADIUS);// + BALL_RADIUS);
      B.setr(ROBOT_RADIUS * 2);
      C.setr(ROBOT_RADIUS * 2);
      D.setr(ROBOT_RADIUS * 1.5);

      B.sett(B.t - PI / 2); C.sett(C.t + PI / 2);
      kickspot.add(EgoBall);
      B.add(EgoBall); C.add(EgoBall);
      D.add(EgoBall);
      // figure out wich quadrant w/ respect to ball

      int quad = Quadrant(EgoBall.t - theirgoal.t);
      if ((quad == 1) || (quad == 4))
        {
          if (Math.abs(EgoBall.t - theirgoal.t) < PI/6)
            {
              result = kickspot;
              result.setr(1.0);
            }
          else
            {
              result = D;
              //     result.setr(1.0);
            }
        }
      else if (quad == 3)
        {result = B; result.setr(1.0);}
      else // quad = 2
        {result = C; result.setr(1.0);}
    }
    return result.t;
  }

  /* Vec2 toward
   *  Given two vectors, return a vector pointing from the first to the
	second
   */
  static Vec2 toward(Vec2 a, Vec2 b)
  {
    Vec2 temp = new Vec2(b.x, b.y);
    temp.sub(a);
    return temp;
  }

  /* ClosestOpponent
   *  Return the position of my closest opponent
   */
  Vec2 ClosestOpponent()
  {
    Vec2 closest = new Vec2( 99999, 0);
    for (int i=0; i< Opponents.length; i++)
      {
        if (Opponents[i].r < closest.r)
          closest = Opponents[i];
      }
    return closest;
  }

  /* ClosestTeammate
   *  Return the position of my closest teammate
   */
  Vec2 ClosestTeammate()
  {
    Vec2 closest = new Vec2( 99999, 0);
    for (int i=0; i< Teammates.length; i++)
      {
        if (Teammates[i].r < closest.r)
          closest = Teammates[i];
      }
    return closest;
  }

  /* haveGoalie
   *  Return true if there is an opponent player in the area where a goalie
   *  would probably be -- ie., guess if they have a dedicated goalie
   */
  boolean haveGoalie()
  {
    double x,y;
    Vec2 dist;
    for (int i=0; i< Opponents.length; i++)
      {
        x = Opponents[i].x; y = Opponents[i].y;
        if ((Math.abs(x-theirgoal.x) < 3 * ROBOT_RADIUS))

 // we commented this out, because the y-value of Opponent position seems
 // to be messed up - perhaps this is a bug in the JavaBots system?

          //       (y < TopGoalPost().y+2*ROBOT_RADIUS) &&
          //       (y > BottomGoalPost().y-2*ROBOT_RADIUS))
          return true;
      }
    return false;
  }

  /* CloseToBall
   *  Return true if the given robot is close to the ball
   */
  boolean CloseToBall(Vec2 robot)
  {
    Vec2 temp = new Vec2(robot.x, robot.y);
    temp.sub(ball);
    return !(temp.r > ROBOT_RADIUS + BALL_RADIUS + .02);
    // Robot radius + how close
  }

  /* ClosestToCenter
   *  Return true if I'm the closest on my team to the center of the field
   */
  boolean ClosestToCenter(Vec2 robot)
  {
    return ClosestTo(robot, new Vec2(0.0,0.0));
  }

  /* ClosestToBall
   *  Return true if I'm the closest on my team to the ball
   */
  boolean ClosestToBall(Vec2 robot)
  {
    //System.out.println("Closest to ball");
    return ClosestTo(robot, ball);
  }

  /* ClosestToMyGoal
   *  Return true if I'm the closest on my team to my goal
   */
  boolean ClosestToMyGoal(Vec2 robot)
  {
    //System.out.println("Closest to goal");
    Vec2 temp = new Vec2(ourgoal.x, ourgoal.y);
    temp.add(robot);
    return ClosestTo(robot, temp);
  }

  /* ClosestTo
   *  Return true if I'm the closest robot on my team to an item
   *  Item's position is absolute, not relative (I think)
   */
  boolean ClosestTo(Vec2 robot, Vec2 item)
  {
    Vec2 temp = new Vec2( robot.x, robot.y);
    temp.sub(item);

    double MyDist = temp.r;
    for (int i=0; i< Teammates.length; i++)
      {
        temp = new Vec2( Teammates[i].x, Teammates[i].y);
        temp.add(robot);
        temp.sub(item);
        double TheirDist = temp.r;
        if (TheirDist < MyDist)
          return false;
      }
    return true;
  }

  /* HaveBall
   *  Return true if we have the ball
   */
  boolean HaveBall(Vec2 robot)
  {
    return (CloseToBall(robot) && BehindBall(robot));
  }

  /* BehindBall
   *  Return true if the ball is between us and the goal-area
   */
  boolean BehindBall(Vec2 robot)
  {
    if (EastTeam(robot))
      {
        if(ball.x >= robot.x) return false;
      }
    else
      {
        if(ball.x <= robot.x) return false;
      }
    Vec2 top = TopGoalPost();
    Vec2 bottom = BottomGoalPost();
    return between(0, NormalizeZero(top.t - ball.t),
                      NormalizeZero(bottom.t - ball.t));
  }

  /* ClearShot
   *  Return true if we are in line to kick the ball into their goal,
   *  angle-wise
   */
  boolean ClearShot(Vec2 robot)
  {
    if(!HaveBall(robot))
      return false;
    double BallDir, TopGoalDir, BottomGoalDir;
    if(EastTeam(robot))
      {
        BallDir = NormalizePI(EgoBall.t);
        TopGoalDir = NormalizePI(toward(me, TopGoalPost()).t);
        BottomGoalDir = NormalizePI(toward(me, BottomGoalPost()).t);
      }
    else
      {
        BallDir = NormalizeZero(EgoBall.t);
        TopGoalDir = NormalizeZero(toward(me, TopGoalPost()).t);
        BottomGoalDir = NormalizeZero(toward(me, BottomGoalPost()).t);
      }
    return between(BallDir, TopGoalDir, BottomGoalDir);
  }

  /* MyBottomGoalPost
   *  Return the egocentric position of the bottom of our goal
   */
  Vec2 MyBottomGoalPost()
  {
    return new Vec2(ourgoal.x, ourgoal.y - GOAL_WIDTH/2);
  }

  /* MyTopGoalPost
   *  Return the egocentric position of the top of our goal
   */
  Vec2 MyTopGoalPost()
  {
    return new Vec2(ourgoal.x, ourgoal.y + GOAL_WIDTH/2);
  }
  /* BottomGoalPost
   *  Return the position of the bottom of their goal
   */
  Vec2 BottomGoalPost()
  {
    return new Vec2(theirgoal.x, theirgoal.y - GOAL_WIDTH/2);
  }

  /* TopGoalPost
   *  Return the position of the top of their goal
   */
  Vec2 TopGoalPost()
  {
    return new Vec2(theirgoal.x, theirgoal.y + GOAL_WIDTH/2);
  }

  /* double NormalizePi
   *  Normalize an angle into the range [0,2*PI]
   */
  static double NormalizePI(double t)
  {
    while(t>PI2) t -= PI2;
    while(t<0) t += PI2;
    return t;
  }

  /* double NormalizeZero
   *  Normalize an angle into the range [-PI,PI]
   */
  static double NormalizeZero(double t)
  {
    while(t>PI) t -= PI2;
    while(t<-PI) t += PI2;
    return t;
  }

  /* int Quadrant
   *  Return the quadrant (I,II,III,IV) of a vector
   */
  static int Quadrant(double theta)
  {
    double t = NormalizePI(theta);
    if ((t > 0) && (t < PI * .5))
      return 1;
    else if ((t >= PI * .5) && (t < PI))
      return 2;
    else if ((t >= PI) && (t < PI * 1.5))
      return 3;
    else //if ((t >= PI * 1.5) && (t < PI2))
      return 4;
  }

  /* TopOfField
   *  Return true if the given vector is in the top half of the field
   */
  boolean TopOfField(Vec2 item)
  {
    return item.y > 0;
  }

  /* EastTeam
   *  Return true if the robot is on the Eastern team
   */
  boolean EastTeam(Vec2 robot)
  {
    return ourgoal.x > 0;
  }

  /* boolean between
   *  Return true if a between b and c
   */
  static boolean between(double a, double b, double c)
  {
    return (a<=Math.max(b,c)) && (a>=Math.min(b,c));
  }

  /* faceOurGoal
   *  Return the heading required to face our goal
   */
  double faceOurGoal()
  {
    return (EastTeam(me) ? 0.0 : PI);
  }

  /* faceTheirGoal
   *  Return the heading required to face their goal
   */
  double faceTheirGoal()
  {
    return (EastTeam(me) ? PI : 0.0);
  }

  /* goalieFunc
   *  The function that controls the goalie
   *   Stay beside the goal, in line with the ball, but not past the ends
   *   of the goal, when the ball is far away.
   *   When it gets close, run to a spot near the ball, horizontally in
   *   line with it, but still between it and the goal
   */
  Vec2 goalieFunc()
  {
    // predict ball position in near future, and always use that value
    Vec2 newEgoBall = predictEgoBall(GOALIE_LOOK_AHEAD+curr_time); //zjh

    // calculate distance from goal to ball - scale in y so that there's
    // an ellipse that covers the whole goal area
    Vec2 balldist = new Vec2(ourgoal.x,ourgoal.y);
    balldist.sub(newEgoBall);
    balldist.sety(balldist.y/0.8);

    Vec2 temp = new Vec2(ourgoal.x, ourgoal.y);
    if (balldist.r < CLOSE_TO_GOAL) {
      // the ball is close.  Run to the spot right in front of it, between
      // it
      // and the goal
      temp.setx(newEgoBall.x +
                (EastTeam(me)?1.0:-1.0) * (ROBOT_RADIUS+BALL_RADIUS));
      temp.sety(newEgoBall.y);

    } else {
      // the ball is far away.  Keep inbetween it and the goal center,
      // but stay in goal
      temp.setx(temp.x + (EastTeam(me) ? -0.9 : 0.9) * (ROBOT_RADIUS));
      temp.sety((temp.y+newEgoBall.y)/2); //zjh
      if (temp.y>MyTopGoalPost().y+ROBOT_RADIUS) // too high
        temp.sety(MyTopGoalPost().y+ROBOT_RADIUS);
      else if (temp.y<MyBottomGoalPost().y-ROBOT_RADIUS) // too low
        temp.sety(MyBottomGoalPost().y-ROBOT_RADIUS);
    }

    // If something went wrong and we got a NaN, then return the center
    // of the goal
    if (invalid(temp)) temp = new Vec2(ourgoal.x, ourgoal.y);

    // Ease to the target spot, so we don't run in circles - since running
    // in circles often results in running around a moving ball
    return easeToRelativeSpot(temp, faceTheirGoal()); //zjh
  }

  /* easeToRelativeSpot
   *  Run to a point, and then face the given direction
   *  We slow down as we get close, so as not to run in circles
   *  We also run slow when facing the wrong direction, so as not to
   *   run too far in the wrong direction when starting out
   */
  Vec2 easeToRelativeSpot(Vec2 Spot, double Heading)
  {
    Vec2 run = new Vec2(Spot.x, Spot.y);

    // calculate factor for speed based on desired-current angle difference
    Vec2 diff = new Vec2(0.0,1.0);
    diff.sett(abstract_robot.getSteerHeading(curr_time)-run.t);
    if (diff.t>PI) diff.sett(PI2-diff.t);
    double factor = (PI-diff.t)/PI;
    if (factor<0.0) factor = 0.0;

    // factor for speed based on closeness to goal spot
    double r = run.r;

    run.setr(factor*r*10+0.2); // can't run slower than 0.2
    if (run.r>1.0) run.setr(1.0); // can't run faster than 1.0

    // We're there!  Stop, and face the given heading
    if (r<CLOSE_TO_SPOT)
      {
        run.sett(Heading);
        run.setr(0.0);
      }
    return run;
  }

  /* easeToAbsoluteSpot
   *  Same as easeToRelativeSpot, except you can put in absolute
   *  coordinates, which is useful for things like:
   *  easeToRelativeSpot(new Vec2(0.0,0.0), faceTheirGoal());
   *  to go to the center of the field
   */
  Vec2 easeToAbsoluteSpot(Vec2 Spot, double Heading)
  {
    return easeToRelativeSpot(new Vec2(Spot.x-me.x, Spot.y-me.y), Heading);
  }

  /* invalid
   *  Check whether any part of a Vector is NaN
   *  Use the fact that:  (q != q  if  q is NaN)
   */
  boolean invalid(Vec2 vector)
  {
    return ((vector.x!=vector.x)||(vector.y!=vector.y));
  }

  /* predictBall
   *  Predict the ball position at a certain time.
   *  Simply extrapolate from last and current position and time
   */
  Vec2 predictBall(long time)
  {
    Vec2 temp = new Vec2(ball.x, ball.y);
    temp.sub(last_ball);

    long diff_time = curr_time - last_time;
    if (diff_time==0.0) diff_time = (long) 0.0001; // don't divide by zero
    temp.setr(temp.r/(diff_time)*(time-curr_time));

    // if it's sitting still, it's going back to the middle
    if(temp.r==0) temp = new Vec2(0.0,0.0);
      else temp.add(ball);
    return temp;
  }

  /* predictEgoBall
   *  Same as predictBall, except ego-centric
   */
  Vec2 predictEgoBall(long time)
  {
    Vec2 temp = predictBall(time);
    temp.sub(me);
    return temp;
  }

  /* blockFunc
   *  The function that steers robot #4 when it is trying
   *  to do our cute little goalie-blocking trick
   *
   *  Run to just below goal, then sweep upwards
   */
  Vec2 blockFunc()
  {
    Vec2 temp = new Vec2(theirgoal.x,
                         BottomGoalPost().y - 3 * ROBOT_RADIUS);
    if (Math.abs(theirgoal.x)<ROBOT_RADIUS+BALL_RADIUS)
      temp.sety(TopGoalPost().y);
    temp.setr(1.0);
    return temp;
  }
}
