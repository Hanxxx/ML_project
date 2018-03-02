/*
 * DumbNode.java
 */

import	java.io.*;
import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;
import	EDU.gatech.cc.is.clay.*;
import	EDU.gatech.cc.is.learning.*;
import	EDU.cmu.cs.coral.abstractrobot.*;

/**
 * A dumb network node.
 * <P>
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2000 CMU
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class DumbNode  extends ControlSystemNetNode
	{
	public final static boolean DEBUG = false;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure()
		{
		}
		
	private boolean done = false;
	/**
	 * Called every timestep to allow the control system to
	 * run.
	 */
	public int takeStep()
		{
		return(CSSTAT_OK);
		}

	public void quit()
		{
		System.out.println("quit()");
		}
	public void trialInit()
		{
		System.out.println("trialInit()");
		}
	public void trialEnd()
		{
		System.out.println("trialEnd()");
                }
	}
