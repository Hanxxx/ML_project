/*
 * BrianTeam.java
 */

import   EDU.gatech.cc.is.util.Vec2;
import   EDU.gatech.cc.is.abstractrobot.*;
import java.lang.Math;

/**
 * Homogeneous team written by Brian McNamara
 * 
 * @author  Brian McNamara (lorgon@cc.gatech.edu)
 * @version $Revision: 1.1 $
 */

/*
Here's a plausible strategy I'd like to implement:

havetheball: am "behind" the ball and very near it
clear: within kicking distance and no robots lie along path
open: no opponent robot within x distance
defense/offense: which side has ball, if unclear, say defense

if( i have the ball ) then
   if( clear shot ) then
      shoot
   else if( i am open ) then
      dribble towards goal
   else if( closest teammate is open && clear pass ) then
      pass to him
   else
      ???
   endif
else
   if( i am closest to ball ) then
      go to "behind" ball (avoid bumping it backwards)
   else
      if( i am closest to ourgoal ) then
         go to point between ball and ourgoal nearest ourgoal
      else if( on defense )
         mark a man (go to nearest open opponent)
      else
         get open (move away from nearest opponent)
      endif
   endif
endif
*/

public class BrianTeam extends ControlSystemSS
{
   double GOAL_WIDTH =  0.42; //?
   double BALL_RADIUS = 0.01; //?

   /**
   Configure the control system.  This method is
   called once at initialization time.  You can use it
   to do whatever you like.
   */
   public void Configure()
   {
      // not used in this example.
   }
      
   public static final double ROBOT_RADIUS = 0.06; // should refer to
                     // actual value, but is hardcoded for convenience

   static String coor( Vec2 v )
   // returns string like "(1.2, 2.0)"
   { 
      String x = String.valueOf(v.x);
      String y = String.valueOf(v.y);
      return "("+x.substring(0,x.indexOf('.')+2)+","+
                 y.substring(0,y.indexOf('.')+2)+")"; 
   }

   String my_name()
   {
      return names[(int)my_id()];
   }

   public long my_id()
   {
      return abstract_robot.getID();
   }

   // Used for debugging
   boolean debug_reminders[] = new boolean[ Debugger.MAX ];

   static long who_closest_to_ball=0;
   static long who_closest_to_ourgoal=0;
   static final String names[] = { "","","","","","",
                                       "orange", "red   ", 
                                       "yellow", "purple", "white " };

   static final boolean
       DEBUG_CLOSEST_TO_OUR_GOAL = false,
       DEBUG_CLOSEST_TO_BALL     = false,
       DEBUG_STATE               = false,
       DEBUG_AROUND              = false,
       dummy                     = false;


   static final int 
      SHOOT     = 0,
      DRIBBLE   = 1,
      RUN       = 2,
      DEFEND    = 3,
      OPEN_OPP  = 4,
      OPEN_TEAM = 5,
      OPEN_BALL = 6,
      CLEAR     = 7,
      foo       = -1;

   static final String state_messages[] = {
      " is going to shoot!",
      " is dribbiling toward the goal.",
      " is running toward the ball.",
      " is defending the goal.",
      " is trying to get open, away from opponent.",
      " is trying to get open, away from teammate.",
      " is trying to get open, toward the ball.",
      " is trying to clear the ball.",
      "" };

   int current_state=0;

   void set_state( int new_state )
   {
      if( DEBUG_STATE )
      {
         if( new_state != current_state )
         {
            current_state = new_state;
            System.out.println( my_name() + state_messages[ new_state ]);
         }
      }
   }

   void debug()
   {
      long my_id = my_id();

      if( DEBUG_CLOSEST_TO_OUR_GOAL )
      {
         if( i_am_closest_to( ourgoal ) )
         {
            if( who_closest_to_ourgoal != my_id )
            {
               who_closest_to_ourgoal = my_id;
               System.out.println( "Robot " + names[(int)my_id] + 
                  " at " + coor( me ) +
                  " is closest to our goal at " + coor( ourgoal ) );
            }
         }
      }
   
      if( DEBUG_CLOSEST_TO_BALL )
      {
         if( i_am_closest_to( ball ) )
         {
            if( who_closest_to_ball != my_id )
            {
               who_closest_to_ball = my_id;
               System.out.println( "Robot " + names[(int)my_id] + 
                  " at " + coor( me ) +
                  " is closest to ball at " + coor( ball ) );
            }
         }
      }
   
//      new Debugger( this, Debugger.TOP_HALF_OF_FIELD );
//      new Debugger( this, Debugger.CLOSEST_TO_BALL );
//      new Debugger( this, Debugger.HAS_THE_BALL );
//      new Debugger( this, Debugger.CLOSE_TO_THE_BALL );
//      new Debugger( this, Debugger.BEHIND_THE_BALL );
//      new Debugger( this, Debugger.HAS_CLEAR_SHOT );
//      new Debugger( this, Debugger.IS_OPEN );
//      new Debugger( this, Debugger.IN_GOALBOX, true );
   }



   // instance variables for convenience across modules
   long curr_time;
   Vec2 me, ball, ourgoal, theirgoal;
   Vec2 ego_ball, ego_ourgoal, ego_theirgoal;
   Vec2 ego_teammates[], ego_opponents[];

   /**
   Find the closest opponent, return ego vector to him
   */
   Vec2 closest_opponent()
   {
      Vec2 closestopponent = new Vec2( -99999, 0 );
      for (int i=0; i< ego_opponents.length; i++)
      {
         if (ego_opponents[i].r < closestopponent.r)
            closestopponent = ego_opponents[i];
      }
      return closestopponent;
   }

   /**
   Find the closest teammate, return ego vector to him
   */
   Vec2 closest_teammate()
   {
      Vec2 closestteammate = new Vec2( -99999, 0 );
      for (int i=0; i< ego_teammates.length; i++)
      {
         if (ego_teammates[i].r < closestteammate.r)
            closestteammate = ego_teammates[i];
      }
      return closestteammate;
   }


   boolean i_am_closest_to_the_ball()
   {
      return i_am_closest_to( ball );
   }

   /**
   Returns true if I am the closest of my ego_teammates to a given location
   */
   boolean i_am_closest_to( Vec2 something )
   // should probably be rewritten to use common functions
   {
      double my_dist, their_dist;
      Vec2 temp;

      Vec2[] ego_teammates = abstract_robot.getTeammates(curr_time);

      for (int i=0; i< ego_teammates.length; i++)
      {
         temp = new Vec2( ego_teammates[i].x, ego_teammates[i].y );
         temp.add( me );
         temp.sub( something );
         their_dist = temp.r;

         temp = abstract_robot.getPosition( curr_time );
         temp = new Vec2( temp.x, temp.y );
         temp.sub( something );
         my_dist = temp.r;

         if( their_dist < my_dist - 0.05 )   // fudge factor
            return false;
      }
      return true;
   }
   


   /**
   Return a Vec2 whose theta is directed from a to b and whose r is dist(a,b)
   */
   static Vec2 toward( Vec2 a, Vec2 b )
   {
      Vec2 temp = new Vec2( b.x, b.y );
      temp.sub( a );
      return temp;
   }
   

   void setInstanceVars()
   {
      // get the current time for timestamps
      curr_time = abstract_robot.getTime();

      // get a list of the positions of our ego_teammates
      ego_teammates = abstract_robot.getTeammates(curr_time);
      ego_opponents = abstract_robot.getOpponents(curr_time);
//should create non-egos, too

      me = abstract_robot.getPosition(curr_time);

      ego_ball = abstract_robot.getBall(curr_time);
      ball = new Vec2( ego_ball.x, ego_ball.y );
      ball.add( me );

      ego_ourgoal = abstract_robot.getOurGoal(curr_time);
      ourgoal = new Vec2( ego_ourgoal.x, ego_ourgoal.y );
      ourgoal.add( me );

      ego_theirgoal = abstract_robot.getOpponentsGoal(curr_time);
      theirgoal = new Vec2( ego_theirgoal.x, ego_theirgoal.y );
      theirgoal.add( me );
   }

   /**
   Called every timestep to allow the control system to
   run.
   */
   public int TakeStep()
   {
      // the eventual movement command is placed here
      Vec2   result;
      Vec2 closestteammate = new Vec2();

      setInstanceVars();

      debug();

      boolean kick_ball = false;
      result = toward( me, ball );

if( false )
{
   result = new Vec2( GOAL_X, -0.4 );
   result = toward( me, result );
}
else
{
      if( i_have_the_ball() )
      {
         if( i_have_a_clear_shot() && toward(me,ego_theirgoal).r < 0.3 ) // reasonable shooting distance
         {
            kick_ball = true;
            result = toward( me, ball );  // dribble toward goal
set_state( SHOOT );
         }
         else if( i_am_in_my_goalbox() )
         {
            kick_ball = true;
            result = toward( me, ball );  // dribble toward goal
set_state( CLEAR );
         }
   

// out since open() fails...
//         else if( open( me, true ) ) // if i am open
//         {
//            result = toward( me, ball );  // dribble toward goal
//         }

//         else if( closest teammate is open && clear pass ) then
//            pass to him
         else // dunno a good strategy
         {
            result = toward( me, ball );  // dribble toward goal
set_state( DRIBBLE );
         }
      }
      else
      {
         if( i_am_closest_to( ball ) )
         {
            result = new Vec2( 1.0, 0 );
            result.sett( toward_behind_ball() );
set_state( RUN );
         }
         else if( i_am_closest_to( ourgoal ) && 
                  ((me.x > 0)==i_am_on_west_team()) )
         {
            result = new Vec2( 0, 0 );         
            result = toward( me, result );
// play goalie
//            result = new Vec2( 1.0, 0 );
//            result.sett( defend_goal() );
//set_state( DEFEND );
         }

//         else if( on defense )
//            mark a man (go to nearest open opponent)
         else
         {
            //get open (move away from nearest opponent)
            result = closest_opponent();
            if( result.r < 2*ROBOT_RADIUS )
            {
               result.sett( result.t + Math.PI );
set_state( OPEN_OPP );
            }
            else
            {
               result = closest_teammate();
               if( result.r < 4*ROBOT_RADIUS )
               {
                  result.sett( result.t + Math.PI );
set_state( OPEN_TEAM );
               }
               else
               {
                  result = toward( me, ball );
set_state( OPEN_BALL );
               }
            }
         }
      }
}

      /*--- Send commands to actuators ---*/
      // set the heading
      abstract_robot.setSteerHeading(curr_time, result.t);

      // set speed at maximum
      abstract_robot.setSpeed(curr_time, 1.0);

      // kick it if we can
      if (abstract_robot.canKick(curr_time))
         if( kick_ball )
            abstract_robot.kick(curr_time);

      // tell the parent we're OK
      return(CSSTAT_OK);
   }

   static final double GOAL_X = 1.0;

   boolean i_am_in_my_goalbox()
   // true if on far side of field
   {
      if( i_am_on_east_team() )
         return me.x > GOAL_X;
      else
         return me.x < -GOAL_X;
   }

   boolean i_am_in_top_half_of_field()
   // lies, tells if in a little higher
   {
      return me.y > 0.1;
   }

   double toward_behind_ball()
   // returns theta direction to go if we want to get behind the ball
   {
      // for now, we'll just be stupid and bump it backwards
      Vec2 result;
      double FUDGE = 0.02;
      if( i_am_on_east_team() )
         result = new Vec2( ball.x + (ROBOT_RADIUS+FUDGE), ball.y ); 
      else
         result = new Vec2( ball.x - (ROBOT_RADIUS+FUDGE), ball.y ); 

      // make egocentric
      result.sub( me );
      if( robot_close_to_ball( me ) && !i_have_the_ball() )
      {
         // i am very near it, but am not behind it
         // try and run around it

         if( i_am_in_top_half_of_field() != i_am_on_east_team() )
         {
if( DEBUG_AROUND )
{
System.out.print( my_name() + " is running around in plus-theta direction" );
System.out.println( " and thinks on top half: " + i_am_in_top_half_of_field() );
}
            return result.t + Math.PI/3;
         }
         else
         {
if( DEBUG_AROUND )
{
System.out.print( my_name() + " is running around in minus-theta direction" );
System.out.println( " and thinks on top half: " + i_am_in_top_half_of_field() );
}
            return result.t - Math.PI/3;
         }
      }
      else
         return result.t;
   }

   Vec2 bottom_goalpost()
   // returns location of their bottom goalpost
   {
      return new Vec2( theirgoal.x, theirgoal.y - GOAL_WIDTH/2 );
   }

   Vec2 top_goalpost()
   // returns location of their top goalpost
   {
      return new Vec2( theirgoal.x, theirgoal.y + GOAL_WIDTH/2 );
   }

   boolean i_have_a_clear_shot()
   // returns true if I have a clear shot on goal
   {
      if( !i_have_the_ball() )
         return false;

      double dir_ball, dir_top_goal, dir_bottom_goal;

      if( i_am_on_east_team() )
      {
         dir_ball = normalize_pi( toward( me, ball ).t );
         dir_top_goal = normalize_pi( toward( me, top_goalpost() ).t );
         dir_bottom_goal = normalize_pi( toward( me, bottom_goalpost() ).t );
      }
      else
      {
         dir_ball = normalize_zero( toward( me, ball ).t );
         dir_top_goal = normalize_zero( toward( me, top_goalpost() ).t );
         dir_bottom_goal = normalize_zero( toward( me, bottom_goalpost() ).t );
      }

      if( between( dir_ball, dir_top_goal, dir_bottom_goal ) )
         return true;
      return false;
   }

   static final double OPENNESS = ROBOT_RADIUS;  // arbitrary value

   boolean i_am_open()
   {
      return open( me, true );
   }

   boolean open( Vec2 robot, boolean my_team )
   // tells if a given robot is "open".  my_team true if on my team
   {
      Vec2 bad[];
      if( my_team ) 
         bad = ego_opponents;
      else
         bad = ego_teammates;
      
      Vec2 temp;
      for( int i=0; i<bad.length; i++ )
      {
         temp = new Vec2( bad[i].x, bad[i].y );

         // correct for fact that "bad" is egocentric
         temp.add( me );

         temp.sub( robot );
//if( names[(int)my_id()].equals("yellow") )
//System.out.println( "temp r is " + temp.r );
         if( temp.r < OPENNESS )
         {
//if( names[(int)my_id()].equals("yellow") )
//System.out.println( "" );
            return false;
         }
      }
//if( names[(int)my_id()].equals("yellow") )
//System.out.println( "" );
      return true;
   }

   boolean clear( Vec2 v )
   // determines if egocentric vector from v is clear of any robots
   {
      int i;
      for( i=0; i<ego_teammates.length; i++ )
         if( robot_intersects( ego_teammates[i], v ) )
            return false;
        
      for( i=0; i<ego_opponents.length; i++ )
         if( robot_intersects( ego_opponents[i], v ) )
            return false;

      return true;
   }

   boolean robot_intersects( Vec2 robot, Vec2 v )
   // returns true if ego-robot intersects ego-v
   {
      double dt = normalize_zero( v.t - robot.t );
      double dist_robot_to_v = robot.r * Math.sin( dt );
      if( dist_robot_to_v > ROBOT_RADIUS )
         return false;
      return true;
   }

   boolean i_have_the_ball()
   // returns true if I'm "behind" the ball and very near it
   {
      if( !robot_close_to_ball( me ) )
         return false;
      if( !i_am_behind_ball() )
         return false;
      return true;
   }

   boolean i_am_behind_ball()
   // returns true if ball is "towards goal" rel to me
   {
      if( i_am_on_east_team() )
      {
         if( ball.x >= me.x ) return false;
      }
      else
      {
         if( ball.x <= me.x ) return false;
      }

      Vec2 top = top_goalpost();
      Vec2 bottom = bottom_goalpost();

      top.sub( me );     // make egocentric
      bottom.sub( me );

      double rel_top = normalize_zero( top.t - ego_ball.t );
      double rel_bot = normalize_zero( bottom.t - ego_ball.t );

      return between( 0, rel_top, rel_bot );
   }
       
   double defend_goal()
   // returns direction we should go to be defending goal
   {
      Vec2 result;
      Vec2 goal = ourgoal;
      if( ball.y > top_goalpost().y )
         goal.sety( top_goalpost().y );
      else if( ball.y < bottom_goalpost().y )
         goal.sety( bottom_goalpost().y );
      else
         goal.sety( ball.y );

      result = toward( goal, ball );
      result.setr( ROBOT_RADIUS );
      result.add( goal );
      result.sub( me );
      return result.t;
   }

   static final double CLOSENESS = 0.05;

   boolean robot_close_to_ball( Vec2 robot )
   {
      Vec2 temp = new Vec2( robot.x, robot.y );
      temp.sub( ball );
//if( names[(int)my_id()].equals("yellow") )
//System.out.println( "temp.r is " + temp.r + " RR is " + ROBOT_RADIUS +
//   " BR is " + BALL_RADIUS + " C is " + CLOSENESS );
      if( temp.r > ROBOT_RADIUS + BALL_RADIUS + CLOSENESS )
         return false;
      return true;
   }

   static double normalize_pi( double t )
   // converts a radian measure to between 0 and 2 pi
   {
      while( t > 2*Math.PI ) { t -= 2*Math.PI; }
      while( t < 0 ) { t += 2*Math.PI; }
      return t;
   }

   static double normalize_zero( double t )
   // converts a radian measure to between -pi and pi
   {
      while( t > Math.PI ) { t -= 2*Math.PI; }
      while( t < -Math.PI ) { t += 2*Math.PI; }
      return t;
   }

   boolean i_am_on_east_team()
   {
      return !i_am_on_west_team();
   }

   boolean i_am_on_west_team()
   {
      return ourgoal.x < 0;
   }

   static boolean between( double a, double b, double c )
   // return true if a between b and c
   {
      return (a<=Math.max(b,c)) && (a>=Math.min(b,c));
   }
}


class Debugger
{
   public static final int 
       HAS_THE_BALL      = 0,
       HAS_CLEAR_SHOT    = 1,
       IS_OPEN           = 2,
       CLOSE_TO_THE_BALL = 3,
       BEHIND_THE_BALL   = 4,
       CLOSEST_TO_BALL   = 5,
       TOP_HALF_OF_FIELD = 6,
       IN_GOALBOX = 7,
       dummy             = -1;

   public static final int MAX = 8;

   final static String yes[] = {
       "gets the ball!",
       "has a clear shot!",
       "has gotten open!",
       "has gotten close to the ball.",
       "is behind the ball.",
       "is closest to the ball.",
       "is in top half of field.",
       "is in his own goalbox.",
       "" };
   final static String no[] = {
       "no longer has the ball.",
       "no longer has a clear shot.",
       "is no longer open!",
       "is no longer close to the ball.",
       "is no longer behind the ball.",
       "is no longer closest to the ball.",
       "is in bottom half of field.",
       "is not in his own goalbox.",
       "" };

   public boolean f()
   {
      switch( which )
      {
         case HAS_THE_BALL:
            return robot.i_have_the_ball();         
         case HAS_CLEAR_SHOT:
            return robot.i_have_a_clear_shot();         
         case IS_OPEN:
            return robot.i_am_open();         
         case CLOSE_TO_THE_BALL:
            return robot.robot_close_to_ball( robot.me );         
         case BEHIND_THE_BALL:
            return robot.i_am_behind_ball();         
         case CLOSEST_TO_BALL:
            return robot.i_am_closest_to_the_ball();         
         case TOP_HALF_OF_FIELD:
            return robot.i_am_in_top_half_of_field();         
         case IN_GOALBOX:
            return robot.i_am_in_my_goalbox();         
         default:
            return false;
      }
   }

   int which;
   boolean highlight=false;
   BrianTeam robot;

   public Debugger( BrianTeam robot, int which )
   {
      this( robot, which, false );
   }

   public Debugger( BrianTeam robot, int which, boolean hl )
   { 
      this.highlight = hl;
      this.robot = robot; 
      this.which = which; 
      
      int id = (int)robot.my_id();

      if( f() )
      {
         if( !robot.debug_reminders[which] )
         {
            if( highlight ) System.out.print( "--> " );
            System.out.println( "Robot " + BrianTeam.names[id] + " " +
                        yes[which] );
            robot.debug_reminders[which] = true;
         }
      }
      else
      {
         if( robot.debug_reminders[which] )
         {              
            if( highlight ) System.out.print( "--> " );
            System.out.println( "Robot " + BrianTeam.names[id] + " " +
                        no[which] );
            robot.debug_reminders[which] = false;
         }
      }
   }
}

