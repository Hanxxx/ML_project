/*
 * FemmeBotsHeteroG.java
 */

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import 	java.lang.Math;

/*
 * Author: 	M. Bernardine Dias
 * 		Robotics Institute
 * 		Carnegie Mellon University
 * Date:	10/05/1999
 *
 *
 
 */


public class FemmeBotsHeteroG  extends ControlSystemSS	{

		private long 	curr_time; 			//What time is it?
		private Vec2	me;				//Where am I in my frame?
		private Vec2	ME;				//Where am I in the global frame?
		private Vec2 	ball;				//Where is the ball?
		private Vec2	their_goal;			//Where is their goal?
		private Vec2	our_goal;			//Where is our goal?
		private Vec2[]	teammates;			//Where are my teammates?
		private Vec2[]	opponents;			//Where are my opponents?
		private Vec2	ball_to_goal;		//How far away is the ball from their goal?
		private Vec2	ball_to_home;		//How far away is the ball from our goal?
		private Vec2	teammate_near_ball;	//Who is the teammate closest 
									//   to the ball?
		private Vec2	ball_to_team;		//A vector from the ball to the teammate closest to the ball;
		private Vec2	opponent_near_ball;	//Who is the opponent closest
									//   to the ball?
		private Vec2	ball_to_opp;		//A vector from the ball to the opponent closest to the ball;
		private Vec2	closest_to_ball;		//Who is closest to the ball?
		private Vec2	just_do_it;			//What should I do?
		private boolean	kick_it;			//Should I kick the ball?
		private Vec2	nearest_teammate;		//Who is the teammate nearest to me?
		private Vec2 	nearest_opponent;		//Who is the opponent nearest to me?			
		private double	set_speed;			//What speed should I move with?
		private int		change;			//An int that helps interchange the angle at which the
									//ball is kicked to the goal
		private long	myID;				//Who am I?
		private Vec2	center;			//Where is the center of the field?
		
	/**
	Configure the Avoid control system.  This method is
	called once at initialization time.  You can use it
	to do whatever you like.
	*/
	public void Configure()
		{
			curr_time = abstract_robot.getTime();
			just_do_it = new Vec2(0,0);
		}
		
	
	/**
	Called every timestep to allow the control system to
	run.
	*/
	public int TakeStep()
		{
		/* --- First we must read our sensor outputs --- */
		//We want to assess where everything is		
		assess_situation();

		/* --- Now we can make some decisions about what to do... --- */
		//First I want to figure out who I am?
		//Based on that I can decide what to do...
		if(myID == 3)
			playCenter();
		if(myID == 2)
			playAttack();
		if(myID == 4)
			playAttack();
		if(myID == 0)
			playGoalKeeper();
		if(myID == 1)
			playDefender();
		//If I am aimed at my goal don't kick the ball		
		if ((kick_it== false) && abstract_robot.canKick(curr_time))
			just_do_it.sett(ball.t + Math.PI);

		/* --- Finally we can send commands to our actuators --- */
		//Set the heading
		abstract_robot.setSteerHeading(curr_time, just_do_it.t);

		//Set the speed
		abstract_robot.setSpeed(curr_time,set_speed);

		//Kick it if we can and should
		if (kick_it && abstract_robot.canKick(curr_time))
			abstract_robot.kick(curr_time);

		//Tell the parent we're OK
		return(CSSTAT_OK);
		}
	
	private void assess_situation()
	{

		//Get current time
		curr_time = abstract_robot.getTime();

		//This is just for conveniance
		me = new Vec2(0,0);
		
		//Who am I?
		myID = abstract_robot.getPlayerNumber(curr_time);
			
		//Initialize vector to where I am in global frame
		ME = abstract_robot.getPosition(curr_time);
		
		//Initialize center of the field
		center = new Vec2(-ME.x,-ME.y);
	
		//Initialize vector to the ball
		ball = abstract_robot.getBall(curr_time);

		//Initialize vector to opponents' goal
		their_goal = abstract_robot.getOpponentsGoal(curr_time);
		
		//Initialize vector to our goal
		our_goal = abstract_robot.getOurGoal(curr_time);

		//Get vectors to all my teammates
		teammates = abstract_robot.getTeammates(curr_time);
			
		//Get vectors to my opponents
		opponents = abstract_robot.getOpponents(curr_time);

		//Get information about who is closest to the ball
		teammate_near_ball = closest_to(ball, teammates);
		opponent_near_ball = closest_to(ball, opponents);
		
		//Initialize to kick the ball straight to the goal
		change = 0;

		//Set my speed to a default of max speed
		set_speed = 1.0;

		//Figure out vectors from the players (excluding self) closest to the ball on each team
		ball_to_team = new Vec2(teammate_near_ball.x, teammate_near_ball.y);
		ball_to_team.sub(ball);
		ball_to_opp = new Vec2(opponent_near_ball.x, opponent_near_ball.y);
		ball_to_opp.sub(ball);

		//Now easy to figure out who is closest to ball
		Vec2 temp = new Vec2(0,0);
		if(ball_to_team.r > ball_to_opp.r)
		{
			closest_to_ball = opponent_near_ball; 
			temp.setr(ball_to_opp.r);
			temp.sett(ball_to_opp.t);
		}
		else
		{
			closest_to_ball = teammate_near_ball; 
			temp.setr(ball_to_team.r);
			temp.sett(ball_to_team.t);
		}
		if(temp.r >= ball.r)
		{
			closest_to_ball = me; 
		}

		//Figure out the distance from the ball to their goal
		ball_to_goal = new Vec2(their_goal.x, their_goal.y);
		ball_to_goal.sub(ball);

		//Figure out the distance from the ball to our goal
		ball_to_home = new Vec2(our_goal.x, our_goal.y);
		ball_to_home.sub(ball);

		//Get information about who is closest to me
		nearest_teammate = closest_to(me, teammates);
		nearest_opponent = closest_to(me, opponents);

		//Initialize boolean that says if we should kick or not
		if(ball.x*their_goal.x >=0)
			kick_it = true;
		else
			kick_it = false;
	}

	private void playDefender()
	{
		if(ball_to_goal.r<(2*ball_to_home.r))
		{
			Vec2 def_pos = new Vec2(0,0);
			def_pos.setx(our_goal.x + abstract_robot.RADIUS * 3);
			def_pos.sety(our_goal.y + abstract_robot.RADIUS * 2);
			if (def_pos.r <abstract_robot.RADIUS* 2)
				set_speed = 0;
			else
			{
				just_do_it = def_pos;
				avoidcollision();
			}
		}
		else
		{
			just_do_it = Calc_block_pos();
			if(ball.r <= abstract_robot.RADIUS)
				just_do_it = their_goal;
		}
		
	}

	
	private void playGoalKeeper()
	//Defend the goal!
	{
		if( ball.x * our_goal.x > 0)
		{
			just_do_it.sett(ball.t);
			kick_it = true;
		}

		else if( (Math.abs(our_goal.x) > abstract_robot.RADIUS * 1.4) ||
			 (Math.abs(our_goal.y) > abstract_robot.RADIUS * 4.25) )

			{
				just_do_it.sett( our_goal.t);
				set_speed = 1;
				avoidcollision();
			}
		else
			{
				if( ball.y > 0)
					just_do_it.sety(7);
				else
					just_do_it.sety(-7);
				if(our_goal.x > 0)
					just_do_it.setx(1);
				else
					just_do_it.setx(-1);

				if( Math.abs( ball.y) < abstract_robot.RADIUS* 0.1)
					set_speed = 0.0;
				else
		 			set_speed = 1.0;
			}

	}
	

	private void playCenter()
	//Stick around the center and try to score 
	{
		if(closest_to_ball == me)
		{
			if(our_goal.r < abstract_robot.RADIUS)
			{
				change = 0;
				just_do_it = new Vec2(their_goal.x, their_goal.y+(abstract_robot.RADIUS*change));
			}
			else if(their_goal.r < abstract_robot.RADIUS*5)
			{
				just_do_it = new Vec2(their_goal.x, their_goal.y+(abstract_robot.RADIUS*change));
				if(change == 0)
					change = 1;
				else
					change = change * (-1);
			}
			else
				just_do_it = Calc_kick_pos();
		}
		else if(ball_to_team.r > ball.r)
		{
			just_do_it = ball;
		}
		else
		{
			if (center.r <abstract_robot.RADIUS* 2)
				set_speed = 0;
			else
			{
				just_do_it = center;
				avoidcollision();
			}
		}
		
	}
	
	private void playAttack()
	//Try to score all the time if the ball is in their half of the field
	//If the ball is on our half wait in assigned position for ball to come back to their half
	{
		if(closest_to_ball == me)
		{
			if(our_goal.r < abstract_robot.RADIUS)
			{
				change = 0;
				just_do_it = new Vec2(their_goal.x, their_goal.y+(abstract_robot.RADIUS*change));
			}
			else if(their_goal.r < abstract_robot.RADIUS*5)
			{
				just_do_it = new Vec2(their_goal.x, their_goal.y+(abstract_robot.RADIUS*change));
				if(change == 0)
					change = 1;
				else
					change = change * (-1);
			}
			else
				just_do_it = Calc_kick_pos();
		}
		else if(ball_to_goal.r <= ball_to_home.r)
		{
			just_do_it = Calc_kick_pos();
		}
		else // if ball is in our half of the field go back to pos and wait for ball
		{
			Vec2 my_pos = new Vec2(0,0);
			if(myID == 2) // go to center-north pos
			{
				my_pos.setx(center.x + abstract_robot.RADIUS*3);
				my_pos.sety(center.y + abstract_robot.RADIUS*3);
			}
			else if(myID == 4) // go to center-south pos
			{
				my_pos.setx(center.x + abstract_robot.RADIUS*3);
				my_pos.sety(center.y - abstract_robot.RADIUS*3);
			}
			if (my_pos.r <abstract_robot.RADIUS* 2)
				set_speed = 0;
			else
			{
				just_do_it = my_pos;
				avoidcollision();
			}
		}

	}


	private Vec2 closest_to(Vec2 ref_pt, Vec2[] contenders)
	//Determines which of the contenders is closest to the reference point
	{
		Vec2 closest = new Vec2(0,0);
		double minDist = 99999;
		Vec2 pt_to_contender = new Vec2(0,0);
		for(int i=0; i<contenders.length; i++)
		{
			pt_to_contender.setr(contenders[i].r);
			pt_to_contender.sett(contenders[i].t);
			pt_to_contender.sub(ref_pt);
			if(pt_to_contender.r < minDist)
			{
				closest.sett(contenders[i].t);
				closest.setr(contenders[i].r);
				minDist = pt_to_contender.r;
			}
		}
		return closest;
	}



	private Vec2 Calc_kick_pos()
	//Calculates a good position from which the ball can be kicked
	{
		Vec2 kick_pos = new Vec2(ball.x, ball.y);
		Vec2 off_goal = new Vec2(their_goal.x, their_goal.y+(abstract_robot.RADIUS*change));
		if(change == 0)
			change = 1;
		else
			change = change * (-1);
		kick_pos.sub(off_goal);
		kick_pos.setr(abstract_robot.RADIUS);
		kick_pos.add(ball);
		return kick_pos;
	}

	private Vec2 Calc_block_pos()
	//Calculates a good position from which the ball can be kicked
	{
		Vec2 block_pos = new Vec2(ball.x, ball.y);
		block_pos.add(our_goal);
		block_pos.setr(block_pos.r*0.5);
		return block_pos;
	}



	private void avoidcollision( )
	{
		// Try to avoid collisions

		if( nearest_teammate.r < abstract_robot.RADIUS*1.5 )
		{
			just_do_it.setx(-nearest_teammate.x);
			just_do_it.sety(-nearest_teammate.y);
			just_do_it.setr(1.0);
		}
		
		else if( nearest_opponent.r < abstract_robot.RADIUS*1.5 )
		{
			just_do_it.setx(-nearest_opponent.x);
			just_do_it.sety(-nearest_opponent.y);
			just_do_it.setr(1.0);
		}

	}


	}
