/*
 * CyeBallSim.java
 */

//package EDU.gatech.cc.is.simulation;

import java.awt.*;
import EDU.gatech.cc.is.util.*;
import EDU.gatech.cc.is.simulation.*;
import EDU.cmu.cs.coral.util.*;
import java.util.*;

/**
 * A soccer ball for RoboCup Soccer.
 * <B>Introduction</B><BR>
 * SoccerBallSim implements a soccer ball for RoboCup
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

public class CyeBallSim extends AttractorSim implements SimulatedObject 
	{
	/*--- keep track of scoring events ---*/
	private	int state = STATE_BEGIN; // this is so the west kicks off.
	private	static final int STATE_BEGIN = 1;
	private	static final int STATE_EAST_SCORE = 2;
	private	static final int STATE_WEST_SCORE = 3;
	private	static final int STATE_PLAY = 4;
	private	int	eastscore = 0;
	private	int	westscore = 0;
	private int colID = -1 ;
	private double lastx, lasty;
	/*--- shot clock time ---*/
	private	static final double TIMEOUT		= 60.0;
	private	double	shotclock = TIMEOUT;

	/*--- average distance recently moved ---*/
	private	double	avg_dist  = 0;
	private	Vec2	last_pos  = new Vec2();

	/*--- dynamics of the ball ---*/
	private	static final double MAX_TRANSLATION	= 1.50;//meters/sec
	private	static final double DECELERATION	= 0.025; //meters/sec/sec
	protected Vec2	velocity = new Vec2(0,0);
        private double MASS_BALL = 0.410; // based on size 5 ball
	private double MASS_ROBOT = 13.0*0.45359; // conversion from pounds to metric
	private double bias = 0.0;
	private long time_passed = 0;

	/*--- set to true to print debug messages ---*/
	public	static final boolean DEBUG = false;
	private Point lastDrawn = new Point(0,0);
	private	CircularBuffer	trail;	// ball's trail

	/**
	 *Instantiate a soccer ball.
	 */
	public CyeBallSim()
		{
		super();
		
		// initial speed is 0
		velocity.sett(Math.random()*Math.PI*2);
		velocity.setr(0.0);
		}

	public void init(double xp, double yp, double t, double r,
		Color f, Color b, int v, int i, long s)
		{
		position = new Vec2(xp,yp);
		RADIUS = r;
		foreground = f;
		background = b;
		visionclass = v;
		setID(i);
		rando = new Random(s);
		rando.nextDouble();
		if (DEBUG) System.out.println("AttractorSim: initialized"
			+" at "+xp+","+yp);
		// reset heading to random
		Random myRand = new Random(System.currentTimeMillis());
		myRand.nextDouble();
		double randx = 2.0*(0.5-myRand.nextDouble())*4.0;
		double randy = 2.0*(0.5-myRand.nextDouble())*2.0;
		position = new Vec2(randx,randy);
		System.out.println("position " + position.x + " " + position.y);
		trail = new CircularBuffer(1000);


		}

	/**
	 * Take a simulated step;
	 */
	public void takeStep(long time_increment, SimulatedObject[] all_objs)
		{
		time_passed += time_increment;
		/*--- remember where we started ---*/
		last_pos = new Vec2(position.x, position.y);
		/*--- take care of state ---*/
		if (state != STATE_PLAY) 
			{
			state = STATE_PLAY;
			}

		/*--- keep pointer to the other objects ---*/
		all_objects = all_objs;

		/*--- keep track of how much time has passed ---*/
		double time_incd = (double)time_increment / 1000;
		shotclock -= time_incd;
		
		/*--- update bias on vision if necessary *---/
		if (time_passed > 4000)
		{
		time_passed = 0;
		bias = (0.5-rando.nextDouble())*10.0; // random number between -5 and 5 degrees;
		bias = bias*2.0*Math.PI/360.0;
	
                /*--- compute a movement step ---*/
                Vec2 mvstep = new Vec2(velocity.x, velocity.y);
                mvstep.setr(mvstep.r * time_incd);
	
                /*--- test the new position to see if in bounds of field ---*/
                Vec2 pp = new Vec2(position.x, position.y);
                pp.add(mvstep);

                /*--- test the new position to see if on top of obstacle ---*/
                pp = new Vec2(position.x, position.y);
                pp.add(mvstep);
		    Circle2 myself = new Circle2(pp, RADIUS);
		boolean not_robot = true;
                for (int i=0; i<all_objects.length; i++)
                       	{
                       	if (all_objects[i].isObstacle() &&
                               	(all_objects[i].getID() != unique_id))
                               	{
				not_robot = true;
                               	Vec2 tmp = all_objects[i].getClosestPoint(pp);
						if (all_objects[i].getVisionClass() == 0) // i.e. ignores robots but not walls
							not_robot = false;
                               	if (all_objects[i].checkCollision(myself)&& not_robot)
							{
							mvstep = new Vec2(0.0,0.0);
							double bounce = Units.BestTurnRad(velocity.t, tmp.t);
							velocity.sett(Math.PI + bounce + tmp.t);
							//	break;
							}
				}
			}
		position.add(mvstep);
	
                /*--- test the new position to see if on top of pushable ---*/
		/* skip this for soccer balls, there are no other
		   pushable objects in a soccer game.*/
                for (int i=0; i<all_objects.length; i++)
                       	{
                       	if (all_objects[i].isPushable() &&
                               	(all_objects[i].getID() != unique_id))
                               	{
                               	Vec2 tmp = all_objects[i].getClosestPoint(pp);
                               	if (all_objects[i].checkCollision(myself))
							{
							  //	mvstep = new Vec2(0.0,0.0);
							((CyeBallSim)all_objects[i]).push_ball(tmp, velocity);

                                       	}
                               	}
                       	}




	


		/*--- decelerate ---*/
		double newvel = velocity.r - (DECELERATION * time_incd);
		if (newvel < 0) newvel = 0;
		velocity.setr(newvel);


		/*--- check shotclock ---*/
		if (shotclock <= 0)
			{
			shotclock = TIMEOUT;
			position.setr(0);
			velocity.setr(0);

			}

		/*--- check for no movement ---*/
// what is the purpose of this?
		last_pos.sub(position);
		avg_dist = avg_dist * 0.9 + Math.abs(last_pos.r);
		if (avg_dist < 0.001)
			{
			shotclock = TIMEOUT;
//			position.setr(0);
			velocity.setr(0);

			}
  
		}

	public Vec2 getClosestPoint(Vec2 from)
		{
		if (picked_up)
			return(new Vec2(99999999,0));
		else 
			{
			Vec2 tmp = new Vec2(position.x, position.y);
			tmp.sub(from);
			if (tmp.r < RADIUS)
				tmp.setr(0);
			else
				tmp.setr(tmp.r-RADIUS);
			lastx = tmp.x + from.x;
			lasty = tmp.y + from.y;
			return(tmp);
			}
		}


	/**
	 * Handle a push.  This is how to kick or push the ball.
	 */

        public void push(Vec2 d, Vec2 v)
                {
		
		  // there is something buggy with this - work on it!

		// update velocity based on principles of conservation of momentum etc.
		double oldVelx = velocity.x;
		double oldVely = velocity.y;

		double CmomentumX = MASS_ROBOT*v.x + MASS_BALL*oldVelx;
		double CmomentumY = MASS_ROBOT*v.y + MASS_BALL*oldVely;
		double Kx = 0.5*MASS_ROBOT*v.x*v.x + 0.5*MASS_BALL*oldVelx*oldVelx; 
		double Ky = 0.5*MASS_ROBOT*v.y*v.y + 0.5*MASS_BALL*oldVely*oldVely; 
		double newVelx;
		double newVely;
		double b = 2.0*MASS_BALL*CmomentumX/MASS_ROBOT;
		double a = MASS_BALL*MASS_BALL/MASS_ROBOT + MASS_BALL;
		double c = CmomentumX*CmomentumX/MASS_ROBOT - 2*Kx;
		
		double tmpVel = b + (Math.sqrt(b*b-4*a*c))/2*a;
		double tmpVelneg =  b - (Math.sqrt(b*b-4*a*c))/2*a;

		double robvel = (CmomentumX-MASS_BALL*tmpVel)/MASS_ROBOT;
		if (b*b < 4*a*c || new Double(robvel).isNaN()) 
		  {
		    newVelx = v.x;
		  }
	    	  else if(dsign(robvel) == dsign(v.x)) 
		  {
		    newVelx = tmpVel;
		  }
		else 
		  {
		    newVelx = tmpVelneg;
		  }
		
		 b = 2.0*MASS_BALL*CmomentumY/MASS_ROBOT;
		a = MASS_BALL*MASS_BALL/MASS_ROBOT + MASS_BALL;
		c = CmomentumY*CmomentumY/MASS_ROBOT - 2*Ky;
		
		tmpVel = b + (Math.sqrt(b*b-4*a*c))/2*a;
		tmpVelneg =  b - (Math.sqrt(b*b-4*a*c))/2*a;

		robvel = (CmomentumY-MASS_BALL*tmpVel)/MASS_ROBOT;
		if (b*b < 4*a*c || new Double(robvel).isNaN()) 
		  {
		    newVely = v.y;
		  }
	    	  else if(dsign(robvel) == dsign(v.y)) 
		  {
		    newVely = tmpVel;
		  }
		else 
		  {
		    newVely = tmpVelneg;
		  }

		velocity = new Vec2(newVelx, oldVely);
		double time = 0.001;
		Vec2 distance = new Vec2(time*velocity.x, time*velocity.y);
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

                }
        public void push_ball(Vec2 d, Vec2 v)
                {
                /*--- move according to the push ---*/
		  //                position.add(d);
		// velocity = new Vec2(v.x,v.y);
		
		// update velocity based on principles of conservation of momentum etc.
		double oldVelx = velocity.x;
		double oldVely = velocity.y;

		double CmomentumX = v.x + oldVelx;
		double CmomentumY = v.y + oldVely;
		double Kx = 0.5*v.x*v.x + 0.5*oldVelx*oldVelx; 
		double Ky = 0.5*v.y*v.y + 0.5*oldVely*oldVely; 
		double newVelx;
		double newVely;
		double b = 2*CmomentumX;
		double a = 2.0;
		double c = CmomentumX*CmomentumX-2.0*Kx;
		
		double tmpVel = b + (Math.sqrt(b*b-4.0*a*c))/2*a;
		double tmpVelneg =  b - (Math.sqrt(b*b-4.0*a*c))/2*a;

		double robvel = CmomentumX-tmpVel;
		if (b*b < 4*a*c || new Double(robvel).isNaN()) 
		  {
		    newVelx = v.x;
		  }
	    	  else if(dsign(robvel) == dsign(v.x)) 
		  {
		    newVelx = tmpVel;
		  }
		else 
		  {
		    newVelx = tmpVelneg;
		  }
		 b = 2*CmomentumY;
		a = 2.0;
		c = CmomentumY*CmomentumY-2.0*Ky;
		
		 tmpVel = b + (Math.sqrt(b*b-4.0*a*c))/2*a;
		 tmpVelneg =  b - (Math.sqrt(b*b-4.0*a*c))/2*a;

		 robvel = CmomentumY-tmpVel;
		if (b*b < 4*a*c || new Double(robvel).isNaN()) 
		  {
		    newVely = v.y;
		  }
	    	  else if(dsign(robvel) == dsign(v.y)) 
		  {
		    newVely = tmpVel;
		  }
		else 
		  {
		    newVely = tmpVelneg;
		  }
		velocity = new Vec2(newVelx, newVely);
		double time = 0.0005;
		Vec2 distance = new Vec2(time*velocity.x, time*velocity.y);
		position.add(distance);

                }

	public double dsign(double a)
          {
	    if (a < 0.0)
	      return -1.0;
	    else if (a > 0.0)
	      return 1.0;
	    else
	      return 0.0;
	  }


	public boolean checkCollision(Circle2 c)
	    {
	    Vec2 closest = getClosestPoint(c.centre); // closest is a vector with origin at centre that leads to closest point on current object
	    if (closest.r <= c.radius) // closest point is within c.radius of c.centre
			{
			return true;
			}
	    else 
			{
			return false;
			}
	    }

        /**
	 * determine if the object is intersecting with a specified polygon.
	 * This is useful for obstacle avoidance and so on.
	 * @param p the polygon which may be intersecting the current object.
	 * @return true if collision detected.
         */

	public boolean checkCollision(Polygon2 p)
		{
		Vec2 vertex1, vertex2, vec1, vector2, closestPt;
		int numberEdges = p.vertices.size(); // n edges if n vertices (as vertex n+1 wraps round to vertex 0)
		double scale;

		for (int i=0;i<numberEdges;i++)
			{
			vertex1 = (Vec2)p.vertices.elementAt(i);
			vertex2 = (Vec2)p.vertices.elementAt((i+1)%numberEdges);
			vertex1.sub(position);
			vertex2.sub(position);
			// if either vertex is within the circles radius you are colliding
			if ((vertex1.r < RADIUS) || (vertex2.r < RADIUS))
				{
				return true;
				} 
			vertex1.add(position);
			vertex2.add(position);
			vec1 = new Vec2(vertex2);
			vec1.sub(vertex1);
			vector2 = new Vec2(position);
			vector2.sub(vertex1);
			scale = ((vec1.x*vector2.x)+(vec1.y*vector2.y))/((vec1.x*vec1.x)+(vec1.y*vec1.y));
			closestPt = new Vec2(scale*vec1.x, scale*vec1.y);
			closestPt.add(vertex1); // absolute position of closest point
			closestPt.sub(position); // position of closest point relative to centre of current object
			if (closestPt.r < RADIUS)
				{
				// now need to check if closestPt lies between vertex1 and vertex2
				// i.e. it could lie on vector between them but outside of them
				if ( (scale > 0.0) && (scale < 1.0) )
					{
					return true;
					}
				}
			}
		return false; // closest point to object on each edge of polygon not within object			
		}



	/**
	 * Draw the soccer ball and display score and shotclock.
	 */
        public void draw(Graphics g, int w, int h,
                double t, double b, double l, double r)
                {
                top =t; bottom =b; left =l; right = r;

                double meterspp = (r - l) / (double)w;
                if (DEBUG) System.out.println("meterspp "+meterspp);
                int radius = (int)(RADIUS / meterspp);
                int xpix = (int)((position.x - l) / meterspp);
                int ypix = (int)((double)h - ((position.y - b) / meterspp));
                if (DEBUG) System.out.println("soccer ball at"+
                        " at "+xpix+","+ypix);

                /*--- draw the ball ---*/
                g.setColor(foreground);
                g.fillOval(xpix - radius, ypix - radius,
                        radius + radius, radius + radius);

                }


	/**
	 * True if the game is underway.  If false, the soccer robots
	 * should reset their positions on the field according to
	 * whether they kick off or not.
	 * @return true if game is underway.
	 */
	public boolean playBall()
		{
		if (state == STATE_PLAY)
			return(true);
		else
			return(false);
		}

        /**
         * Draw the objects's State.
         */

	
        public void drawState(Graphics g, int w, int h,
                double t, double b, double l, double r)
                {
                double meterspp = (r - l) / (double)w;
                if (DEBUG) System.out.println("meterspp "+meterspp);
                int x1pix = (int)((lastx - l) / meterspp);
                int y1pix = (int)((double)h - ((lasty - b) / meterspp));
                if (DEBUG) System.out.println("line at"+
                        " at "+x1pix+","+y1pix);

                /*--- draw the oval ---*/
                g.setColor(background);
                g.fillOval(x1pix-2, y1pix-2, 4, 4);

		// draw vector from ball to goal location
		Vec2 vec = new Vec2(4.6, 0.0);
		Vec2 tmpVec = new Vec2(position);
		vec.sub(tmpVec);
		double dis = 0.1 - 0.02;
		vec = new Vec2((dis/vec.r)*vec.x, (dis/vec.r)*vec.y);
		int xt2pix = (int)((vec.x + position.x -l)/meterspp);
		int yt2pix = (int)((double)h - ((vec.y + position.y -b)/meterspp));
		
		int xt1pix = (int)((0.0 + position.x -l)/meterspp);
		int yt1pix = (int)((double)h - ((0.0 + position.y -b)/meterspp));

		g.drawLine(xt1pix, yt1pix, xt2pix, yt2pix);


		/*--- record ball's trail ---*/

                /*--- record the point ---*/
                Point p = new Point(xt1pix,yt1pix);
                if ((lastDrawn.x != p.x)||(lastDrawn.y != p.y))
                        trail.put(p);
                lastDrawn = p;

                /*--- get the list of all points ---*/
                Enumeration point_list = trail.elements();

                /*--- draw the trail ---*/
                g.setColor(foreground);
                Point from = (Point)point_list.nextElement();
                while (point_list.hasMoreElements())
                        {
                        Point next = (Point)point_list.nextElement();
                        g.drawLine(from.x,from.y,next.x,next.y);
                        from = next;
                        }

                }


	/**
	 * True if it is east's turn to kick off.
	 * @return true if it is east's turn to kick off.
	 */
	public boolean eastKickOff()
		{
		if (state == STATE_WEST_SCORE)
			return(true);
		else
			return(false);
		}


	/**
	 * True if it is west's turn to kick off.  This occurs
	 * at the begining of the game, and after east scores.
	 * @return true if it is west's turn to kick off.
	 */
	public boolean westKickOff()
		{
		if ((state == STATE_EAST_SCORE)||(state == STATE_BEGIN))
			return(true);
		else
			return(false);
		}


	/**
	 * True if it is west just scored.
	 * @return true if west just scored.
	 */
	public boolean westJustScored()
		{
		if (state == STATE_WEST_SCORE)
			return(true);
		else
			return(false);
		}


	/**
	 * True if it is east just scored.
	 * @return true if east just scored.
	 */
	public boolean eastJustScored()
		{
		if (state == STATE_EAST_SCORE)
			return(true);
		else
			return(false);
		}
	}
