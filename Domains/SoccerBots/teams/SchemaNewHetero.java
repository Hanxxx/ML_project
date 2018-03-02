/*
 * SchemaNewHetero.java
 */

import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;
import  EDU.gatech.cc.is.clay.*;

/**
 * This is an example of a simple control system for soccer robots
 * designed using Clay.
 * The strategy is to get behind the ball, then move to it, while
 * keeping away from teammates.
 * One of the robots remains near the goal to act as a goalie.
 * <P>
 * For detailed information on how to configure behaviors, see the
 * <A HREF="../../EDU/gatech/cc/is/clay/docs/index.html">Clay page</A>.  
 * <P>
 * <A HREF="../../EDU/gatech/cc/is/COPYRIGHT.html">Copyright</A>
 * (c)1997, 1998 Tucker Balch
 *
 * @author Tucker Balch
 * @version $Revision: 1.1 $
 */

public class SchemaNewHetero extends ControlSystemSS
        {
        public final static boolean DEBUG = false;
            
        private NodeVec2        steering_configuration;
        private NodeBoolean     kick_configuration;
        private NodeInt         state_monitor;

        private final static double CLOSE = .05;

        /**
         * Configure the SchemaDemo control system using Clay.
         */
        public void configure()
                {
                if (DEBUG) System.out.println("SchemaDemo.Configure()");
                abstract_robot.setObstacleMaxRange(3.0);
                
                //======
                //Perceptual schemas
                //======
                // the teammates sensed by the robot
                NodeVec2Array 
                PS_TEAMMATES = new va_Teammates_r(abstract_robot);

                // the opponents sensed by the robot
                NodeVec2Array 
                PS_OPPONENTS = new va_Opponents_r(abstract_robot);

                //the closest opponent to the robot
                NodeVec2
                PS_CLOSEST_OPP = new v_Closest_va(PS_OPPONENTS);

                // our egocentric pos (to pass into things that need it)
                NodeVec2
                PS_BOT = new v_FixedPoint_(0, 0);

                // the ball
                NodeVec2
                PS_BALL = new v_Ball_r(abstract_robot);

                // good place to kick from
                NodeVec2
                PS_SWEET_SPOT = new v_SweetSpot_r(abstract_robot);

                //calculate distance of teammates from ball
                NodeVec2Array
                PS_TMM_TO_BALL = new va_Subtract_vav(PS_TEAMMATES, PS_BALL);
                
                //the closest teammate to the ball
                NodeVec2
                PS_CLOSEST_TMM_TO_BALL = new v_Closest_va(PS_TMM_TO_BALL);

                // our goal
                NodeVec2
                PS_OUR_GOAL = new v_OurGoal_r(abstract_robot);

                // their goal
                NodeVec2
                PS_THEIR_GOAL = new v_TheirGoal_r(abstract_robot);

                // point halfway between ball and goal
                NodeVec2
                PS_HALFWAY = new v_Average_vv(PS_OUR_GOAL,PS_BALL);

                        // point halfway between ball and nearest opponent
                NodeVec2
                PS_BLOCK = new v_Average_vv(PS_CLOSEST_OPP,PS_BALL);

                        // point halfway between ball and goal
                NodeVec2
                PS_DOWN = new v_Average_vv(PS_THEIR_GOAL,PS_BALL);

        


                //======
                // PERCEPTUAL FEATURES
                //======
                // tells us if we are behind the ball or not
                NodeBoolean
                PF_BEHIND_BALL = new b_BehindBall_r(abstract_robot);

        
                // tells us if we can kick the ball or not
                NodeBoolean
                PF_CAN_KICK    = new b_CanKick_r(abstract_robot);

                //tells us if the ball is in our possession (hopefully)
                //by telling us if we're close to the ball 
                NodeBoolean
                PF_CLOSE_TO_BALL = new b_Close_vv(CLOSE,PS_BALL,PS_BOT);
                
                NodeBoolean
                PF_CLOSEST_TO_BALL = new b_Shorter_vv(PS_BALL, PS_CLOSEST_TMM_TO_BALL);
        
                //======
                // PERCEPTUAL STATE
                //======
                // builds a state from bits
                i_Merge_ba
                STATE = new i_Merge_ba();
                STATE.embedded[2] = PF_BEHIND_BALL;
                STATE.embedded[1] = PF_CLOSE_TO_BALL;
                STATE.embedded[0] = PF_CLOSEST_TO_BALL;
                state_monitor = STATE;
                

                //======
                // MOTOR SCHEMAS
                //======
                // a motor schema to avoid our teammates
                NodeVec2 
                MS_AVOID_TEAMMATES 
                        = new v_Avoid_va(1.0, abstract_robot.RADIUS+0.1, 
                                PS_TEAMMATES);

                // a motor schema to avoid opponents
                NodeVec2 
                MS_AVOID_OPPONENTS 
                        = new v_Avoid_va(0.2, abstract_robot.RADIUS-0.01, 
                                PS_OPPONENTS);

        
        

                // a motor schema to go to the ball
                NodeVec2 
                MS_MOVE_TO_BALL
                        = new v_LinearAttraction_v(0.0, 0.0, PS_BALL);

                NodeVec2 
                MS_MOVE_TO_BLOCK_POS
                        = new v_LinearAttraction_v(0.0, 0.0, PS_BLOCK);
                
                // a motor schema to go to the sweetspot
                NodeVec2 
                MS_MOVE_TO_SWEET_SPOT
                        = new v_LinearAttraction_v(0.0, 0.0, PS_SWEET_SPOT);

                // a motor schema to go to halfway
                NodeVec2 
                MS_MOVE_TO_HALFWAY
                        = new v_LinearAttraction_v(0.0, 0.0, PS_HALFWAY);

                // swirl around the ball
                NodeVec2 
                MS_SWIRL_BALL
                        = new v_Swirl_vv(0.5, 0.0, PS_BALL, PS_HALFWAY);

                // swirl around the closest opponent
                NodeVec2 
                MS_SWIRL_BLOCK
                        = new v_Swirl_vv(0.5, 0.0, PS_CLOSEST_OPP, PS_BLOCK);


                // a motor schema to go to the goal area
                NodeVec2 
                MS_MOVE_TO_BACK
                        = new v_LinearAttraction_v(0.3, 0.2, PS_OUR_GOAL);

                // a motor schema to go to the other teams's goal area
                NodeVec2 
                    MS_MOVE_DOWN
                    = new v_LinearAttraction_v(0.3, 0.2, PS_THEIR_GOAL);


                //======
                // MOVE TO BALL ASSEMBLAGE
                //======
                v_StaticWeightedSum_va 
                AS_MOVE_TO_BALL = new v_StaticWeightedSum_va();

                AS_MOVE_TO_BALL.weights[0]  = 0.8;
                AS_MOVE_TO_BALL.embedded[0] = MS_AVOID_TEAMMATES;
                
                AS_MOVE_TO_BALL.weights[1]  = 1.0;
                AS_MOVE_TO_BALL.embedded[1] = MS_MOVE_TO_SWEET_SPOT;

                //======
                // BLOCK CLOSEST OPP ASSEMBLAGE
                //======
                v_StaticWeightedSum_va AS_BLOCK_CLOSEST_OPP = 
                    new v_StaticWeightedSum_va();
                AS_BLOCK_CLOSEST_OPP.weights[0]  = 0.8;
                AS_BLOCK_CLOSEST_OPP.embedded[0] = MS_AVOID_TEAMMATES;
                
                AS_BLOCK_CLOSEST_OPP.weights[1]  = 1.2;
                AS_BLOCK_CLOSEST_OPP.embedded[1] = MS_MOVE_TO_BLOCK_POS;
                
                AS_BLOCK_CLOSEST_OPP.weights[2]  = 1.0;
                        AS_BLOCK_CLOSEST_OPP.embedded[2] = MS_SWIRL_BLOCK;



                //======
                // GET BEHIND BALL ASSEMBLAGE
                //======
                v_StaticWeightedSum_va AS_GET_BEHIND_BALL = 
                        new v_StaticWeightedSum_va();
                AS_GET_BEHIND_BALL.weights[0]  = 0.8;
                AS_GET_BEHIND_BALL.embedded[0] = MS_AVOID_TEAMMATES;

                AS_GET_BEHIND_BALL.weights[1]  = 1.2;
                AS_GET_BEHIND_BALL.embedded[1] = MS_SWIRL_BALL;
                
                AS_GET_BEHIND_BALL.weights[2]  = 1.0;
                AS_GET_BEHIND_BALL.embedded[2] = MS_MOVE_TO_HALFWAY;


        
                //======
                // MOVE TO BACKFIELD
                //======
                v_StaticWeightedSum_va AS_MOVE_TO_BACKFIELD = 
                        new v_StaticWeightedSum_va();
        

                AS_MOVE_TO_BACKFIELD.weights[0]  = 1.0;
                AS_MOVE_TO_BACKFIELD.embedded[0] = MS_MOVE_TO_BALL;
                
                AS_MOVE_TO_BACKFIELD.weights[1]  = 1.2;
                AS_MOVE_TO_BACKFIELD.embedded[1] = MS_MOVE_TO_BACK;


                 //======
                // MOVE DOWNFIELD
                //======
                v_StaticWeightedSum_va AS_MOVE_DOWNFIELD = 
                        new v_StaticWeightedSum_va();

                 AS_MOVE_DOWNFIELD.weights[0]  = 1.0;
                 AS_MOVE_DOWNFIELD.embedded[0] = MS_AVOID_OPPONENTS;
                
                 AS_MOVE_DOWNFIELD.weights[1]  = 1.2;
                 AS_MOVE_DOWNFIELD.embedded[1] = MS_MOVE_DOWN;
   
                
                //======
                // SPECIFY CONFIGURATIONS
                //======
                // define the steering configuration
                 v_Select_vai 
                     STEERING = new v_Select_vai(STATE);
                 
                 
                 
                 if ( abstract_robot.getPlayerNumber(-1) == 0){
                     // I'm not the closest teammate to the ball
                     STEERING.embedded[0] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[1] = AS_MOVE_TO_BACKFIELD;
                     STEERING.embedded[2] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[3] = AS_MOVE_TO_BACKFIELD;
                     // I'm the closest teammate to the ball
                     STEERING.embedded[4] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[5] = AS_MOVE_TO_BALL;
                     STEERING.embedded[6] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[7] = AS_MOVE_TO_BALL;
                 }
                 else  {
                     // I'm not the closest teammate to the ball
                     STEERING.embedded[0] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[1] = AS_MOVE_TO_BALL;
                     STEERING.embedded[2] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[3] = AS_MOVE_TO_BALL;
                     // I'm the closest teammate to the ball
                     STEERING.embedded[4] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[5] = AS_MOVE_TO_BALL;
                     STEERING.embedded[6] = AS_GET_BEHIND_BALL;
                     STEERING.embedded[7] = AS_MOVE_DOWNFIELD;
                 }
                 steering_configuration = STEERING;
                 
        
                 kick_configuration =  new b_And_bb(PF_CAN_KICK, PF_BEHIND_BALL);
                }
                
        
        /**
        Called every timestep to allow the control system to
        run.
        */
        public int takeStep()
                {
                Vec2    result;
                boolean kick_result;
                long    curr_time = abstract_robot.getTime();

                // Get the decisions from the configurations
                if (steering_configuration == null)
                        System.out.println("it is null");
                result = steering_configuration.Value(curr_time);
                if (result.r > 1.0)
                        result.setr(1.0);
                kick_result = kick_configuration.Value(curr_time);

                // Send them to the robot
                abstract_robot.setSteerHeading(curr_time, result.t);
                abstract_robot.setSpeed(curr_time, result.r);
                if (kick_result) abstract_robot.kick(curr_time);
                


                // Display the action for the user
                int state = state_monitor.Value(curr_time);
                String msg = "blank";
                if (state == 0)
                    msg = "get behind ball";
                else if (state == 1)
                    msg = "move to ball";
                else if (state == 2)
                    msg = "get behind ball";
                else if (state == 3)
                    msg = "move downfield";
                if (state == 4)
                        msg = "get behind ball";
                else if (state == 5)
                    msg = "block opponent";
                else if (state == 6)
                    msg = "block opponent";
                else msg = "move to backfield";
                
                abstract_robot.setDisplayString(msg);

                // Check for scoring event
                if (abstract_robot.getJustScored(curr_time) != 0)
                        {
                        System.out.println("score: " 
                                + abstract_robot.getJustScored(curr_time));
                        }

                // Tell parent we're OK
                return(CSSTAT_OK);
                }
        }
