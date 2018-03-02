/*
 * b_And_bb.java
 */

import java.lang.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.util.Vec2;

/**
 * Ands the values of two boolean nodes.
 * This module is a node used in Clay to configure soccer behaviors.
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


public class b_And_bb extends NodeBoolean
        {
        /** 
         * Turn debug printing on or off.
         */
        public static final boolean DEBUG = Node.DEBUG;
        private NodeBoolean     embedded1;
        private NodeBoolean     embedded2;
        /**
         * Instantiate a b_And_bb schema.
         * that provides hardware support.
         */
        public b_And_bb(NodeBoolean b1, NodeBoolean b2)
                {
                if (DEBUG) System.out.println("b_And_bb: instantiated");
                embedded1 = b1;
                embedded2 = b2;
                }

        long    last_t = 0;
        boolean last_val = false;
        /**
         * Return the anded value of the embedded nodes.
         * @param timestamp long, only get new information 
         *        if timestamp > than last call or timestamp == -1.
         * @return true if behind the ball, false otherwise.
         */
        public boolean Value(long timestamp)
                {
                if (DEBUG) System.out.println("b_And_bb: Value()");

                if ((timestamp > last_t) || (timestamp == -1))
                        {
                        if (timestamp != -1) last_t = timestamp;

                        boolean tmp1 = embedded1.Value(timestamp);
                        boolean tmp2 = embedded2.Value(timestamp);

                        last_val = tmp1 && tmp2;
                        }
                return(last_val);
                }
        }


