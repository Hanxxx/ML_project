/*
 * DTeam.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	java.lang.Math;

/**
 * A reflexive, heuristic, heterogenous soccer team.
 *
 * @author David H. Johnson
 * @version $Revision: 1.1 $
 */


public class DTeam extends ControlSystemSS
{

	private long	curr_time;		//What time is it?
	private long	mynum;			//Who am I?
	private double	rotation;		//What direction am I pointing?

	private Vec2	ball;			//Where is the ball?
	private Vec2	ourgoal;		//Where is our goal?
	private Vec2	theirgoal;		//Where is their goal?
	private Vec2[]	teammates;		//Where are my teammates?
	private Vec2[]	opponents;		//Where are my opponents?

						//Where is the closest...
	private Vec2	closest_team;		//Teammate?
	private Vec2	closest_opp;		//Opponent?
	private Vec2	closest_to_ball;	//Teammate to the Ball?


	private Vec2	move;			//Move in move.t direction
						//  with speed move.r
	private boolean	kickit;			//Try to kick it


	// what side of the field are we on? -1 for west +1 for east
	private static int SIDE;

	// a vector pointing to me.
	private static final Vec2 ME = new Vec2(0,0);

	// restated here for convenience
	private final double ROBOT_RADIUS = abstract_robot.RADIUS;

	private static final boolean DEBUG = false;

	/**
	Configure the control system.  This method is
	called once at initialization time.  You can use it
	to do whatever you like.
	*/
	public void configure()
	{
		curr_time = abstract_robot.getTime();
		if( abstract_robot.getOurGoal(curr_time).x < 0)
			SIDE = -1;
		else
			SIDE = 1;

		move = new Vec2(0,0);
	}
		
	
	/**
	Called every timestep to allow the control system to
	run.
	*/
	public int takeStep()
	{
		Vec2 closest = new Vec2(0,0);
		update_env();

		/*--- Goalie ---*/
		if( mynum == 0)
		{
			play_goalie();
		}

		/*--- Backup ---*/
		else if( mynum == 1)
		{
			play_backup();
		}

		/*--- Offside ---*/
		else if( mynum == 2)
		{
			play_offside();
		}

		/*--- Designated Driver ---*/
		else if( mynum == 3)
		{
			drive_ball();
		}

		/*--- Center ---*/
		else
			play_center();

		/*--- Send commands to actuators ---*/
		// set the heading
		abstract_robot.setSteerHeading(curr_time, move.t);

		// set the speed
		abstract_robot.setSpeed(curr_time, move.r);

		// maybe kick it
		if (kickit && abstract_robot.canKick( curr_time))
			abstract_robot.kick(curr_time);

		// tell the parent we're OK
		return(CSSTAT_OK);
	}

	private void update_env( )
	{
		Vec2 closest;
		Vec2 temp = new Vec2(0,0);

		/*--- bookkeeping data ---*/
		// get the current time for timestamps
		curr_time = abstract_robot.getTime();

		// get my player id
		mynum = abstract_robot.getPlayerNumber(curr_time);

		/*--- sensor data ---*/
		// get vector to the ball
		ball = abstract_robot.getBall(curr_time);

		// get vector to our and their goal
		ourgoal = abstract_robot.getOurGoal(curr_time);
		theirgoal = abstract_robot.getOpponentsGoal(curr_time);

		// get a list of the positions of our teammates
		teammates = abstract_robot.getTeammates(curr_time);

		// get a list of the positions of the opponents
		opponents = abstract_robot.getOpponents(curr_time);

		// get closest data
		closest_team = closest_to( ME, teammates);
		closest_opp = closest_to( ME, opponents);

		closest = closest_to( ball, teammates);

		temp.sett( closest.t);
		temp.setr( closest.r);
		
		temp.sub(ball);

		if( temp.r > ball.r)
			closest_to_ball = ME;
		else
			closest_to_ball = closest;

		rotation = abstract_robot.getSteerHeading( curr_time);

		/*--- default actuator control ---*/
		// set movement data: rotation and speed;
		move.sett(0.0);
		move.setr(0.0);

		// set kicking
		kickit = false;

	}

	private void play_goalie()
	{
		// if the ball is behind me try to kick it out
		if( ball.x * SIDE > 0)
		{
			move.sett( ball.t);
			move.setr( 1.0);
			kickit = true;
		}

		// if i'm outside the goal area go back toward the goal
		else if( (Math.abs(ourgoal.x) > ROBOT_RADIUS * 1.4) ||
			 (Math.abs(ourgoal.y) > ROBOT_RADIUS * 4.25) )

		{
			move.sett( ourgoal.t);
			move.setr( 1.0);
		}

		// stay between the ball and the goal
		else
		{
			if( ball.y > 0)
				move.sety( 7);
			else
				move.sety( -7);

			move.setx( (double)SIDE);

			if( Math.abs( ball.y) < ROBOT_RADIUS * 0.15)
				move.setr( 0.0);
			else
				move.setr( 1.0);
		}
	}

	private void play_offside( )
	{
		// the other team's goalie is whoever is closest to the goal
		Vec2 goalie = closest_to( theirgoal, opponents);

		// find the point just behind the "goalie" 
		// in the way of their goal
		theirgoal.sub( goalie);
		theirgoal.setr( ROBOT_RADIUS );
		theirgoal.add( goalie);

		move.sett( theirgoal.t);
		move.setr( 1.0);

		// if you aren't blocking the "goalie" then don't collide
		if( goalie != closest_opp)
			avoidcollision();
	}

	private void play_backup( )
	{

		Vec2 backup = new Vec2(0,0);

		if( closest_to_ball == ME)
			drive_ball();
		else

		// if i'm not closest to the ball, set up a position 3
		// robot radii behind the ball
		{
			backup.sett( ball.t);
			backup.setr( ball.r);
			backup.setx( backup.x + ROBOT_RADIUS * 3 * SIDE);
			get_behind( backup, theirgoal);
			avoidcollision();
		}

	}

	private void play_center()
	{
		Vec2 center = new Vec2(0,0);

		// find the center
		center = abstract_robot.getPosition( curr_time);
		center.setr( -center.r);

		if( closest_to_ball == ME)
			drive_ball();
		else

		// if i'm not closest to the ball stick around the center
		// and wait for a fast break
		{
			get_behind( center, theirgoal);
			avoidcollision();
		}
	}
			


	private void drive_ball()
	{
		// if i'm behind the ball (oriented toward the goal) then
		// start charging the goal
		if( behind_point( ball, theirgoal) && ball.t < ROBOT_RADIUS * 4)
		{
			move.sett( theirgoal.t);
			move.setr( 1.0);

			// if i'm within 15x ROBOT_RADII away from and aiming
			// relatively at the goal try to kick the ball
			if( (Math.abs( rotation - theirgoal.t) < Math.PI/8) &&
			    (theirgoal.r < ROBOT_RADIUS * 15))
				kickit = true;
		}
		else

		// otherwise get behind the ball and avoid colliding with
		// other players
		{
			get_behind( ball, theirgoal);
			avoidcollision();
		}
	}

/* This was implemented to see how a single (non-random) robot would act.
   I left it in because a random team might not be so bad
	private void play_random( )
	{
		move.sett( Math.random() * 2 * Math.PI);
		move.setr( Math.random() * abstract_robot.MAX_TRANSLATION);
		kickit = true;
	}
*/

	private Vec2 closest_to( Vec2 point, Vec2[] objects)
	{
		double dist = 9999;
		Vec2 result = new Vec2(0, 0);
		Vec2 temp = new Vec2(0, 0);

		for( int i=0; i < objects.length; i++)
		{

			// find the distance from the point to the current
			// object
			temp.sett( objects[i].t);
			temp.setr( objects[i].r);
			temp.sub( point);

			// if the distance is smaller than any other distance
			// then you have something closer to the point
			if(temp.r < dist)
			{
				result = objects[i];
				dist = temp.r;
			}
		}

		return result;
	}

	private void get_behind( Vec2 point, Vec2 orient)
	{
		Vec2 behind_point = new Vec2(0,0);
		double behind = 0;
		double point_side = 0;

		// find a vector from the point, away from the orientation
		// you want to be
		behind_point.sett( orient.t);
		behind_point.setr( orient.r);

		behind_point.sub( point);
		behind_point.setr( -ROBOT_RADIUS*1.8);

		// determine if you are behind the object with respect
		// to the orientation
		behind = Math.cos( Math.abs( point.t - behind_point.t));

		// determine if you are on the left or right hand side
		// with respect to the orientation
		point_side = Math.sin( Math.abs( point.t - behind_point.t));

		// if you are in FRONT
		if( behind > 0)
		{
			// make the behind point more of a beside point
			// by rotating it depending on the side of the
			// orientation you are on
			if( point_side > 0)
				behind_point.sett( behind_point.t + Math.PI/2);
			else
				behind_point.sett( behind_point.t - Math.PI/2);
		}

		// move toward the behind point
		move.sett( point.t);
		move.setr( point.r);
		move.add( behind_point);

		move.setr( 1.0);

	}

	private boolean behind_point( Vec2 point, Vec2 orient)
	{

		// you are behind an object relative to the orientation
		// if your position relative to the point and the orientation
		// are approximately the same
		if( Math.abs( point.t - orient.t) < Math.PI/10) 
			return true;
		else
			return false;
	}

	private void avoidcollision( )
	{
		// an easy way to avoid collision

		// first keep out of your teammates way
		// if your closest teammate is too close, the move away from
		if( closest_team.r < ROBOT_RADIUS*1.4 )
		{
			move.setx( -closest_team.x);
			move.sety( -closest_team.y);
			move.setr( 1.0);
		}

		// if the closest opponent is too close, move away to try to
		// go around
		else if( closest_opp.r < ROBOT_RADIUS*1.4)
		{
			move.setx( -closest_opp.x);
			move.sety( -closest_opp.y);
			move.setr( 1.0);
		}

	}


/*  This function has not yet been integrated

	private void nearpath( Vec2 goal, Vec2[] objects)
	{
		double mindist = 9999;
		double dist;
		double theta;
		Vec2 result = new Vec2( 0, 0);
		Vec2 temp = new Vec2(0, 0);

		// find which object of objects[] is nearest
		// line that makes up your goal
		for( int i=0; i < objects.length; i++)
		{
			theta = Math.abs( objects[i].t - goal.t);

			if( theta > Math.PI/2)
			{
				if( objects[i].r < mindist)
				{
					result = objects[i];
					mindist = objects[i].r;
				}
			}
			else if( objects[i].r * Math.cos( theta) > goal.r)
			{
				temp.sett( objects[i].t);
				temp.setr( objects[i].r);
				temp.sub( goal);

				if( temp.r < mindist)
				{
					result = objects[i];
					mindist = temp.r;
				}
			}
			else
			{
				dist = objects[i].r * Math.sin( theta);
				if( dist < mindist)
				{
					result = objects[i];
					mindist = dist;
				}
			}
		}
	}
*/
	private void debug( String message)
	{
		if( DEBUG)
			System.out.println( mynum + ":  " + message);
	}
	

}
