/*
 * b_ResetableBoolean_.java
 */

//package EDU.gatech.cc.is.clay;
import EDU.gatech.cc.is.clay.*;

import java.lang.*;

/**
 * Return true if the value of embedded node equals a set integer value.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../clay/docs/index.html">Clay page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)1997, 1998 Tucker Balch
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */


public class b_ResetableBoolean_ extends NodeBoolean
	{
	/** 
	Turn debug printing on or off.
	*/
	public static final boolean DEBUG = Node.DEBUG;

	    protected boolean _value;

	/**
	   Don't ask, don't tell.
	*/
	public b_ResetableBoolean_(boolean initialVal)
		{
		if (DEBUG) System.out.println("b_ResetableBoolean_: instantiated.");
		_value=initialVal;
		}

	boolean	last_val = false;
	long	lasttime = 0;
	/**
	Return a boolean indicating if the embedded schema
	output is equal to a desired value. 
	@param timestamp long, only get new information if timestamp > than last call
                or timestamp == -1.
	@return true if equal, false if not.
	*/
	    public boolean Value(long timestamp)
	    {
                if (DEBUG) System.out.println("b_ResetableBoolean_: Value()");
 
                if ((timestamp > lasttime)||(timestamp == -1))
		    {
                        /*--- reset the timestamp ---*/
                        if (timestamp > 0) lasttime = timestamp;
 
			last_val = _value;
		    }
		return last_val;
	    }
	    /**
	       Sets the internal stored boolean to the given value.
	     */
	    public void setValue(boolean b)
	    {
		_value=b;
	    }


}

























































































































