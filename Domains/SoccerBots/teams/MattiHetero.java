/*
 * MattiHetero.java
 */

// Eventually commented out so I don't have to put the file in the teams folder

import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
//Clay not used

/**
 * Autonomous Multirobot Systems: Project 1
 * 
 * Built from BasicTeam.java
 *
 * See "MattiDescription" for information on strategy
 *
 * @author Matthias Felber
 * @version $Revision: 1.1 $
 */


public class MattiHetero extends ControlSystemSS
	{
	
	
	double DEFENSE_RADIUS = 0.6;
	double PASS_DISTANCE = 0.4;
  	double GOAL_WIDTH = .45;  // .5 - 2 x ball_radius
  	double BALL_RADIUS = .02;

	
// declare the variables here (instead of in takestep) to access 
// them in the other member functions
	private long	curr_time;		//What time is it?
	private long	mynum;			//Who am I?
	private double	rotation;		//What direction am I pointing?

	private Vec2  me;				// Just a vector pointing to me i.e. (0,0)
	private Vec2	ball;			//Where is the ball?
	private Vec2	ourgoal;		//Where is our goal?
	private Vec2	theirgoal;		//Where is their goal?
	private Vec2	theirleftpost;		//Where is their left goalpost?
	private Vec2	theirrightpost;		//Where is their right goalpost?
	
	private Vec2	center;			// the center of the field
	private Vec2[]	teammates;		//Where are my teammates?
	private Vec2[]	opponents;		//Where are my opponents?
	private Vec2  closestteammate;
	
	private Vec2 leftwing;			// strategic positions for getting passes
	private Vec2 rightwing;
	private Vec2 backwing;

	// what side of the field are we on? -1 for west +1 for east
	private int SIDE;

	// restated here for convenience
	private final double ROBOT_RADIUS = abstract_robot.RADIUS;
	


	/**
	Configure the control system.This method is
	called once at initialization time. You can use it
	to do whatever you like.
	*/
	public void Configure()
		{
		curr_time = abstract_robot.getTime();
		if( abstract_robot.getOurGoal(curr_time).x < 0)
			SIDE = -1;
		else
			SIDE = 1;
			
		leftwing = new Vec2(-0.1*SIDE, -0.4*SIDE);	// vector to the left 
		rightwing = new Vec2(-0.1*SIDE,0.4*SIDE);
		backwing = new Vec2(SIDE*0.4,0);	// some place behind the ball
		
		me=new Vec2(0.0,0.0);	

		}
		
	
	/**
	Called every timestep to allow the control system to
	run.
	*/
	public int TakeStep()
		{
		// the eventual movement command is placed here
		Vec2	result = new Vec2(0,0);

		// get the current time for timestamps
		curr_time = abstract_robot.getTime();


		/*--- Get some sensor data ---*/
		// get vector to the ball and to the center of the field
		ball = abstract_robot.getBall(curr_time);
		center = abstract_robot.getPosition(curr_time);
		
		// get vector to our and their goal
		 ourgoal = abstract_robot.getOurGoal(curr_time);
		 theirgoal = abstract_robot.getOpponentsGoal(curr_time);
		 theirgoal.setx(theirgoal.x+SIDE*ROBOT_RADIUS);
		 theirleftpost = new Vec2(theirgoal.x,theirgoal.y-SIDE*GOAL_WIDTH/2.5);
		 theirrightpost = new Vec2(theirgoal.x,theirgoal.y+SIDE*GOAL_WIDTH/2.5);
		 

		// get a list of the positions of our teammates
		teammates = abstract_robot.getTeammates(curr_time);

		/* get a list of the positions of the opponents */
		opponents = abstract_robot.getOpponents(curr_time); 
		
		// mynum not used in Homogeneous team
		mynum = abstract_robot.getPlayerNumber(curr_time);
		

		// find the closest teammate
		closestteammate = new Vec2(99999,0);
		for (int i=0; i< teammates.length; i++)
			{
			if (teammates[i].r < closestteammate.r)
				closestteammate = teammates[i];
			}


		Vec2 behind_ball=behindball();		// vector pointing behind the ball (from the ball) 
		leftwing = new Vec2(behind_ball);	// vector to the left 
		leftwing.rotate(-Math.PI/2);
		leftwing.setr(Math.min(distance(ball,theirgoal)/4+0.15,0.3));
		rightwing = new Vec2(behind_ball);	// and to the right
		rightwing.rotate(Math.PI/2);
		rightwing.setr(Math.min(distance(ball,theirgoal)/4+0.15,0.3));
		
		if (ball.x*SIDE<0)						// ball on the other half 
			backwing = new Vec2(SIDE*0.4,0);	// some place behind the ball
		else 
		{
			backwing = new Vec2(behind_ball);
			backwing.setr(0.25);
		};


		result=find_around_ball(behind_ball);	// default
		abstract_robot.setDisplayString("to ball");

		
		if (i_am_closest_to(ball)) 	// get the ball and attack (highest priority)
		{
			if (i_have_ball())
			{	
		//		behind_ball.sett(get_best_shot()+Math.PI);  
				result=find_around_ball(behind_ball);	
			}
			else
			{	
				result=find_around_ball(behind_ball);
			}
			if (shot_value(ball.t)>0)
				result = ball;	// kick straight if possible

		}
		else // I am not closest
		{
		if (mynum==0) 		// play goalie
		{	
			Vec2 temp=new Vec2(ball);
			temp.sub(ourgoal);	// ball to goal
			if (temp.x*SIDE<-2*DEFENSE_RADIUS)
			{	// to allow left and right to take their role
				temp.setr(DEFENSE_RADIUS);
				temp.add(ourgoal);
				temp.sety(ball.y);	/* follow the ball, so the goalie doesnt become left-
or
									rightmost, preventing the others to choose this role */
			}
			else if (temp.r>DEFENSE_RADIUS)
			{ // move out a bit if the ball is far from our goal
			  // prevents some of those mean goalie blocking behaviors
				temp.setr(Math.min(temp.r-DEFENSE_RADIUS,DEFENSE_RADIUS));
				temp.add(ourgoal);
			}
			else
			{	// stay inside the goal
				if (temp.y>0.25) 
					temp.sety(0.25);
				if (temp.y<-0.25)
					temp.sety(-0.25);
				temp.setx(0);
				temp.add(ourgoal);
			}
			
			behind_ball=temp;
			
			result=behind_ball;
			abstract_robot.setDisplayString("Goalie");
		}
		else //not closest and not backmost
		{
		if ((mynum==2)&&(-SIDE*toward(center,ball).y<0.55))	
		// play left wing if ball not too close to left border
		{
				{
				behind_ball.add(leftwing);
				behind_ball.add(ball);
				result= behind_ball;
				abstract_robot.setDisplayString("leftwing");
				}
		} else  // not leftmost either
		 if  ((mynum==4)&&(SIDE*toward(center,ball).y<0.55	))
		 // play right wing
		{
			{
				behind_ball.add(rightwing);
				behind_ball.add(ball);
				result= behind_ball;
				abstract_robot.setDisplayString("rightwing");
			}

		}
		else	// otherwise
		{ 
			// avoid closest teammate and cover the back of the ball
			behind_ball.add(ball);	
			behind_ball.add(backwing);	
			abstract_robot.setDisplayString("backwing");
			if (closestteammate.r < 4*ROBOT_RADIUS)
				result=find_around_player(closestteammate,behind_ball);
		}
		}
		}

		 ;

		// brake if close to the ball and looking in the wrong direction 
		// brake if very close to the result and dont have the ball
		// otherwise set speed at maximum
		if (((result.r< 2*ROBOT_RADIUS) && 
		(Math.abs(normalizeZero(abstract_robot.getSteerHeading(curr_time)-result.t
))>Math.PI/2))||
		((result.r<BALL_RADIUS)&&(ball.r>2*ROBOT_RADIUS)))
		{
		abstract_robot.setSpeed(curr_time,  0.5);
//		System.out.println(mynum+": radius: "+result.r+ball.r);
		}
		else 		
			abstract_robot.setSpeed(curr_time,  1.0);

		/*--- Send commands to actuators ---*/
		// set the heading
		abstract_robot.setSteerHeading(curr_time, result.t);

		double shotvalue =shot_value(abstract_robot.getSteerHeading(curr_time));
		// kick it if we can and if it's worth it
		if (abstract_robot.canKick(curr_time))
			if (shotvalue>0)
			{
				abstract_robot.kick(curr_time);
				System.out.println("shot: "+shotvalue);
			}

		// tell the parent we're OK
		return(CSSTAT_OK);
		}
		
		
		
		
		
		
		
		
		
	/** Evaluates the shot_value function at several points and returns the
best direction
	* Evaluated points: goal, leftpost, rightpost, teammates in front , (past
closest opponent in front)
	* 
	* The function is actually not used, since it doesn't really improve the
performance
	* If I had some better ball control it could be interesting to use it again
	*/
	private double get_best_shot()
	{
		double tempangle=0,tempvalue=0,maxvalue=-1,best_shot=theirgoal.t;
		String who = new String("???");
		
		/*Evaluate the best pass to my teammates*/
		for (int i=0; i< teammates.length; i++)
		if (teammates[i].x*SIDE<0.3)	// He is almost in front of me
		{
			Vec2 tempvect  = new Vec2(frontofteammate(teammates[i]));
			tempvect.add(teammates[i]);
			tempvect= toward(ball,tempvect);
			tempangle=tempvect.t;
			tempvalue = shot_value(tempangle);
			if (tempvalue > maxvalue) 
			{
				maxvalue = tempvalue;
				best_shot=tempangle;
				who = new String("Pass"+i);
			}
		}
		/* Evaluate at the center of their goal */
		Vec2 tempvect  = new Vec2(theirgoal);
		tempvect.sub(ball);
		if ((tempvalue=shot_value(tempvect.t))>maxvalue) {
			best_shot = tempvect.t;
			maxvalue=tempvalue;
			who = new String("goal");
			}
			
		/* Evaluate at the left goalpost */
		tempvect  = new Vec2(theirleftpost);
		tempvect.sub(ball);
		if (shot_value(tempvect.t)>maxvalue) {
			best_shot = tempvect.t;
			maxvalue=tempvalue;
			who = new String("LP");
			}			

		/* Evaluate at the right goalpost */
		tempvect  = new Vec2(theirrightpost);
		tempvect.sub(ball);
		if (shot_value(tempvect.t)>maxvalue) {
			best_shot = tempvect.t;
			maxvalue=tempvalue;
			who = new String("RP");

			}

		/* Chosing the closest opponent in front of me */
		/*	Vec2 closestopp= new Vec2();
		tempvalue=9999;
		for (int i=0; i< opponents.length; i++)
			if (opponents[i].x*SIDE<0)	// He is in front of me
			{
				if (opponents[i].r<tempvalue)	
				{
					closestopp = new Vec2(opponents[i]);
					tempvalue=closestopp.r;
				}
			}
		closestopp.sub(ball);
		// left side of the opponent
		tempangle = Math.asin((ROBOT_RADIUS+BALL_RADIUS)/closestopp.r);
		if shotvalue(tempangle+
		Math.abs(Math.sin(norm_anglediff(theta,opponents[i].t)))*opponents[i].r
<ROBOT_RADIUS)	// negative value
			value = -1;	
*/


//		Vec2[] shots=Vec2[];
//		System.out.println("bestshot: "+best_shot + " value: "+ maxvalue);
		abstract_robot.setDisplayString("find shot: "+who);
		return best_shot;
	}
	
	/** Get some sort of a quality factor for a shot in the indicated direction

	*/
	private double shot_value(double theta)
	{
		double value=0;
		double temp_value=0;
		for (int i=0; i< teammates.length; i++)
		{
		if (teammates[i].r <PASS_DISTANCE)	// positve = attractive value
		{
			if (normalizeZero(theirgoal.t-teammates[i].t)<0)	//teammate on the left side of goal?
			{
				temp_value=normalizeZero(teammates[i].t-theta);
				if (temp_value>ROBOT_RADIUS/teammates[i].r) // pointing in front of the teammate?
				{
					temp_value=1-temp_value/(Math.PI/6);
					if (temp_value>0)
						value+=temp_value;
				}
			}
			else	// teammate on the right side
			{
				temp_value=normalizeZero(teammates[i].t-theta);
				if ( temp_value<-ROBOT_RADIUS/teammates[i].r) // pointing in front of the teammate?
				{
					temp_value=1+temp_value/(Math.PI/6);
					if (temp_value>0)
						value+=temp_value;
				}
			}
		}	
		} //for 
		
		// Pointing towards their goal (between the goal-Posts?
		temp_value =(normalizeZero( ball.t-theirgoal.t));
		if ((temp_value<normalizeZero(theirleftpost.t-theirgoal.t))&&	
			temp_value>normalizeZero(theirrightpost.t-theirgoal.t))
			if (theirgoal.r<PASS_DISTANCE)
				value+=3.5;
			else value+=0.7;
		
		
		// Pointing towards an opponent -> overrides all the previous values 
		for (int i=0; i< opponents.length; i++)
		{
		if (Math.abs(Math.sin(normalizeZero(opponents[i].t-theta)))*opponents[i].r
<ROBOT_RADIUS)	// negative value
			value = -1;	
		}
//		System.out.println("shot: "+value+"  at "+theta);
		return value;
	}
		
		
	/* *  Given two vectors, return a vector pointing from the first to the
second
   	*/
	  private Vec2 toward(Vec2 a, Vec2 b)
	  {
	    Vec2 temp = new Vec2(b.x, b.y);
	    temp.sub(a);
	    return temp;
	  }
		
	/**
	Compute a vector, that points behind the ball (from the ball)
	Choosing offensive or defensive strategy, depending on the position of the
ball
	
	*/
	private Vec2 behindball()
	{
		
		Vec2 behind_ball = toward(ball,ourgoal);
		if (behind_ball.r >DEFENSE_RADIUS) 
		{	// choose offensive strategy if ball far from our goal
			//  --> behind is calculated with respect to the other goal:
			behind_ball = toward(theirgoal,ball);
			behind_ball.setr(ROBOT_RADIUS*1.0);
	//		 abstract_robot.setDisplayString("Attack");
		}
		else 
		{
			behind_ball.setr(abstract_robot.RADIUS*1.5);
//		 abstract_robot.setDisplayString("Defense");
		};
		return behind_ball;
	}	
	
	/** Returns the location in front of the teammate towards the goal*/
	private Vec2 frontofteammate(Vec2 player)
	{
		Vec2 before = toward(player,theirgoal);
		before.setr(ROBOT_RADIUS*2);
		return before;
	}



   /**  Normalize an angle into the range [-PI,PI]
   */
	private double normalizeZero(double angle)
	{
		while (angle>Math.PI) 
			angle -= 2*Math.PI;
		while (angle<-Math.PI)
			angle += 2*Math.PI;				// range -PI .. PI		
		return angle;
	}			
	
	
	/**
	returns a vector to get around the ball to the behindspot without bumping
the ball
	*/
	private Vec2 find_around_ball (Vec2 spotfromball)
	{
		Vec2 behindspot = new Vec2(spotfromball);
		behindspot.setr(ROBOT_RADIUS*0.3);
		behindspot.add(ball);
		if
(Math.abs(normalizeZero(spotfromball.t-toward(ball,me).t))>Math.PI/2)	// test if really behind
		{	if (normalizeZero(behindspot.t-ball.t) >0)	// pass on the left of ball 
{	
				behindspot.rotate(1.2*Math.asin(Math.min(1,ROBOT_RADIUS/ball.r)));
				 abstract_robot.setDisplayString("Turn 'left");
			}
			else
			{
				behindspot.rotate(-1.2*Math.asin(Math.min(1,ROBOT_RADIUS/ball.r)));
				 abstract_robot.setDisplayString("Turn 'right");
			};
		}
		else	// behind the ball behindspot.r < ball.r
		if ((behindspot.r<ROBOT_RADIUS)&&i_have_ball())
		{
			double dribble_cheat = 0.5;
			if (normalizeZero(behindspot.t-ball.t)>0)
				behindspot.rotate(dribble_cheat);
			else
				behindspot.rotate(-dribble_cheat);
			 abstract_robot.setDisplayString("Dribble");
	}
		return behindspot;
	}
	
	
	/** Same thing as find_around_ball, but to get around another player
	*/
		private Vec2 find_around_player (Vec2 player,Vec2 spot)
	{
		Vec2 behindspot = new Vec2(toward(player,spot));
		behindspot.add(player);
		if
(Math.abs(normalizeZero(toward(player,spot).t-toward(player,me).t))>Math.PI/
2)	// test if really behind
			if (normalizeZero(behindspot.t-player.t) >0)	// pass on the left of player
			{	
				behindspot.rotate(1.2*Math.asin(Math.min(1,1.9*ROBOT_RADIUS/player.r)));
				 abstract_robot.setDisplayString("avoid left");
			}
			else
			{
				behindspot.rotate(-1.2*Math.asin(Math.min(1,1.9*ROBOT_RADIUS/player.r)))
;
				 abstract_robot.setDisplayString("avoid right");
			};
	return behindspot;
	}
	
	

	/**
	Do we control the ball?
	*/
	private boolean we_have_ball()
	 // 
	 {
	 	return
(distance(ball,closest_teammate_to(ball))<distance(ball,closest_opponent_to(
ball)));
	 }

	/**
	Am I at the ball and behind the ball?
	*/
	private boolean i_have_ball()
	 // 
	 {
	 	Vec2 spot = new Vec2(ball);
	 	
	 	return(
i_am_closest_to(ball)&&(Math.abs(normalizeZero(theirgoal.t-ball.t))<Math.PI/
2));
	 }
	 
	 private double distance(Vec2 spot1,Vec2 spot2)
	 {
	 	if ((spot1==null)||(spot2==null))
	 		return 9999;
	 	else
	 	{
	 	Vec2 temp = new Vec2(spot1);
	 	temp.sub(spot2);
	 	return temp.r;
	 	}
	 }
	 
	/**
	 Who is closest to the point (from our team)?
	*/
	private Vec2 closest_teammate_to(Vec2 spot)
	 // 
	 {
	 	double closest_r; 
		Vec2 closest = null;
	        closest_r=9999; //distance(me,spot);
		for (int i=0; i< teammates.length; i++)
		{
			if (distance(teammates[i],spot)<closest_r)
			{
				closest = teammates[i];
				closest_r=distance(closest,spot);
			}
		}
		return closest;
	}
	
		
	private Vec2 closest_opponent_to(Vec2 spot)
	 // 
	 {
	 	double closest_r; 
		Vec2 closest = me;
	        closest_r=distance(me,spot);
		for (int i=0; i< opponents.length; i++)
		{
			if (distance(opponents[i],spot)<closest_r)
			{
				closest = opponents[i];
				closest_r=distance(closest,spot);
			}
		}
		return closest;
	}

	/**Am I closest to this spot? 
	      (overestimating the  distances of the other teammates, so several can
be closest) */
	private boolean i_am_closest_to(Vec2 spot)
	{
		Vec2 behind = behindball();
		behind.setr(2*ROBOT_RADIUS);
		behind.add(ball);
		return ((distance(closest_teammate_to(spot),spot)>spot.r*0.9)||
		(distance(closest_teammate_to(behind),behind)>behind.r*0.9));
	}


	/**
	am I leftmost?
	*/
	private boolean i_am_leftmost()
	 // 
	 {
	Vec2 leftmost = me;
	for (int i=0; i< teammates.length; i++)
			{	// slightly egoistic view. tends to see itself on the left even if its not exactly the case
			if (SIDE*teammates[i].y< SIDE*leftmost.y-ROBOT_RADIUS/2)
				leftmost = teammates[i];
			}
	return me==leftmost;
	 }
	 
	/**
	am I rightmost ?
	*/
	private boolean i_am_rightmost()
	 // 
	 {
	Vec2 rightmost = me;
	for (int i=0; i< teammates.length; i++)
			{
			if (SIDE*teammates[i].y-ROBOT_RADIUS/2> SIDE*rightmost.y)
				rightmost = teammates[i];
			}

	return me==rightmost;
	 }
	
	/**
	am I backmost ?
	*/
	private boolean i_am_backmost()
	// 
	 {
	Vec2 backmost = me;
	for (int i=0; i< teammates.length; i++)
			{
			if (SIDE*teammates[i].x> SIDE*backmost.x)
				backmost = teammates[i];
			}

	return me==backmost;
	 }

	/**
	am I frontmost ?
	*/
	private boolean i_am_frontmost()
	 // 
	 {
	Vec2 frontmost = me;
	for (int i=0; i< teammates.length; i++)
			{
			if (SIDE*teammates[i].x< SIDE*frontmost.y)
				frontmost = teammates[i];
			}

	return me==frontmost;
	 }

	
	//from DTeam
	private void debug( String message)
	{
		//if( DEBUG)
			System.out.println( ":  " + message);
	}
	

		
		
		
	}
