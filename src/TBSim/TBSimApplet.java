/*
 * TBSimApplet.java
 */

package TBSim;

import java.applet.*;

/**
 * Runs TBSim as an Applet.
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
 * @see TBSim#TBSim
 */

public class TBSimApplet extends Applet
	{
	private TBSim jbs;

	public void init()
		{
		String[] argsin = new String[10];
		int i;

		for (i=0; i<10; i++)
			{
			try
				{
				argsin[i] = this.getParameter("arg"+i);
				}
			catch(Exception e) {break;}
			if (argsin[i] == null) break;
			}
		String[] args = new String[i];

		for (int j = 0; j<i; j++)
			args[j] = argsin[j];

		jbs = new TBSim();
		jbs.main(args);
		}
	}
