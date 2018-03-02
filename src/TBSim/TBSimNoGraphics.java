/*
 * TBSimNoGraphics.java
 */

package TBSim;

import java.io.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.*;

/**
 * Application that runs a control system in simulation with no graphics.
 * <P>
 * To run this program, first ensure you have set your CLASSPATH correctly,
 * then type "java TBSim.TBSimNoGraphics".
 * <P>
 * For more detailed information, see the
 * <A HREF="docs/index.html">TBSim page</A>.
 * <P>
 * <A HREF="../EDU/cmu/cs/coral/COPYRIGHT.html">Copyright</A> 
 * (c)1997, 1998 Tucker Balch and GTRC
 * (c)1998 Tucker Balch and Carnegie Mellon University
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class TBSimNoGraphics
	{
	private static SimulationCanvas simulation;
	private static String	dsc_file;

        /**
	 * Main for TBSimNoGraphics.
         */
	public static void main(String[] args)
		{
		/*--- check the arguments ---*/
		if (args.length == 1)
			{
			if (args[0].equalsIgnoreCase("-version"))
				{
				System.out.println(
					TBVersion.longReport());
				System.exit(0);
				}
			else
				{
				dsc_file=args[0];

				/*--- tell the simulation to load and run ---*/
				System.out.println(
					TBVersion.shortReport());
				simulation =
					new SimulationCanvas(null,0,0,dsc_file);
				simulation.reset();
				if (simulation.descriptionLoaded())
					// only if loaded ok.
					{
					simulation.start();
					}
				}
			}
		else
			{
			System.out.println(
				"usage: TBSimNoGraphics [-version] descriptionfile");
			}
		}
	}
