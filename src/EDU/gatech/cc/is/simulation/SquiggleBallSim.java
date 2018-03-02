/*
 * SquiggleBallSim.java
 */

package EDU.gatech.cc.is.simulation;

import java.awt.*;

import TBSim.SimulationCanvas;

import EDU.gatech.cc.is.clay.NodeVec2;
import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.util.Units;
import forage.multiforage;
import EDU.gatech.cc.is.abstractrobot.ControlSystemMFN150;
/**
 * implements a moving attractor for JavaBotSim simulation.
 * <P>
 * Copyright (c)2000 Tucker Balch
 *
 * @author Tucker Balch
 * @version $Revision: 1.2 $
 */

public class SquiggleBallSim extends AttractorSim implements SimulatedObject
	{
	private	static final double MAX_TRANSLATION = 0.8;//0.9;//.8//1.0;//0.61; // 2 feet/second//0.5
	private	static final double TIMEOUT = 5; 
	private Vec2	velocity = new Vec2(0,MAX_TRANSLATION);
	public	static final boolean DEBUG = false;
	public static Vec2 SquiggleBalldircection;
	

	public SquiggleBallSim()
		{
		super();
		velocity.sett(Math.random()*Math.PI*2);
		}

	double	accumtime = 0;
	/**
	 * Take a simulated step;
	 */
	public void takeStep(long time_increment, SimulatedObject[] all_objs)
		{
		if ((picked_up != true)&&(deposited != true))
			{
			double time_incd = (double)time_increment / 1000;
			accumtime += time_incd;
			if (accumtime >= TIMEOUT)
				{
				accumtime = 0;
				velocity.sett(Math.random()*Math.PI*2);
				velocity.setr(MAX_TRANSLATION);
				}
	
			/*--- keep pointer to the other objects ---*/
			all_objects = all_objs;
	
                	/*--- compute a movement step ---*/
                	Vec2 mvstep = new Vec2(velocity.x, velocity.y);
                	mvstep.setr(mvstep.r * time_incd);
                	
                	/*---检查是否被发现----*/
                	Vec2 pp = new Vec2(position.x, position.y);
                	pp.add(mvstep);
                	
            
                	SquiggleBalldircection = pp;
                	
                	
                	
                	/*
                	Vec2 tmp1;
                	double[] dis = new double[4];//改这里
                	double Vmin;
                	int min = 0;
                	if (time_incd != 0)
                	{
                		if ((SimulationCanvas.judgement[0]==1)||(SimulationCanvas.judgement[1]==1)||(SimulationCanvas.judgement[2]==1)||(SimulationCanvas.judgement[3]==1))//改这里
                		{
                			for (int i=0; i<4; i++)//改这里
                				{
                				tmp1 = SimulationCanvas.SquiggleBall[i];
                				dis[i] = (tmp1.x-pp.x)*(tmp1.x-pp.x)+(tmp1.y-pp.y)*(tmp1.y-pp.y);
                				dis[i] =  Math.sqrt(dis[i]);
                				}
                			//选择dis中最小的那个
                		        Vmin  = 9999999999f;
                		        for (int i = 0; i < 4; i++)//改这里
                		        {
                		        	if (dis[i] < Vmin)
                		        	{
                		        		Vmin = dis[i];
                		        		min = i;
                		        	}
                		        } 		       
                			Vec2 direction = new Vec2(pp.x-SimulationCanvas.SquiggleBall[min].x,pp.y-SimulationCanvas.SquiggleBall[min].y);
                			velocity.sett(direction.t);
                			velocity.setr(MAX_TRANSLATION);
                            //mvstep = new Vec2(velocity.x, velocity.y);
                        	//mvstep.setr(mvstep.r * time_incd);
                        	//pp.add(mvstep);
                		}
                	}
               */
               
                	/*--- test the new position to see if in bounds ---*/
                	pp = new Vec2(position.x, position.y);
                	pp.add(mvstep);
                	if (pp.x+RADIUS > right)
                        	{
                        	velocity.setx(0);
                        	mvstep.setx(0);
				velocity.sett(Math.random()*Math.PI+(Math.PI/2));
                        	}
                	if (pp.x-RADIUS < left)
                	{
                	velocity.setx(0);
                	mvstep.setx(0);
		velocity.sett(Math.random()*Math.PI+(Math.PI/2*3));
                	}
                	if (pp.y+RADIUS > top)
                        	{
                        	velocity.sety(0);
                        	mvstep.sety(0);
				velocity.sett(Math.random()*Math.PI+(Math.PI));
                        	}
                	if (pp.y-RADIUS < bottom)
                	{
                	velocity.sety(0);
                	mvstep.sety(0);
		velocity.sett(Math.random()*Math.PI);
                	}
	
                	/*--- test the new position to see if on top of obstacle ---*/
                	pp = new Vec2(position.x, position.y);
                	boolean moveok = true;
                	pp.add(mvstep);
                	for (int i=0; i<all_objects.length; i++)
                        	{
                        	if (all_objects[i].isObstacle() &&
                                	(all_objects[i].getID() != unique_id))
                                	{
                                	Vec2 tmp = all_objects[i].getClosestPoint(pp);
                                	Vec2 asd = tmp;
                                	if (tmp.r <= 0)
                                        	{
                                        	moveok = false;
						velocity.sett(Math.random()*Math.PI*2);
                                        	break;
                                        	}
                                	}
                        	}
                        	
                	if (moveok) position.add(mvstep);
	
                	/*--- test the new position to see if on top of pushable ---*/
                	for (int i=0; i<all_objects.length; i++)
                        	{
                        	if (all_objects[i].isPushable() &&
                                	(all_objects[i].getID() != unique_id))
                                	{
                                	Vec2 tmp = all_objects[i].getClosestPoint(pp);
                                	if (tmp.r < RADIUS)
                                        	{
                                        	tmp.setr(RADIUS - tmp.r);
                                        	all_objects[i].push(tmp, velocity);
                                        	}
                                	}
                        	}
			}
		}
	}
