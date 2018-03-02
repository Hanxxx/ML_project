
import   EDU.gatech.cc.is.util.Vec2;
import   EDU.gatech.cc.is.abstractrobot.*;
import java.lang.Math;

/**
 * Hetrogeneous JavaSoccer player written by Brian McNamara
 * 
 * @author Brian McNamara (lorgon@cc.gatech.edu)
 * @version $Revision: 1.1 $
 */


public class BriSpec
extends BrianTeam
{

   public int TakeStep()
   {
      setInstanceVars();
      Vec2 result;
      boolean kick_ball=false;

      if( abstract_robot.getPlayerNumber(curr_time)==0 )
      {
         // play goalie
         result = new Vec2( 1.0, 0 );
         result.sett( defend_goal() );
         kick_ball = true;

         /*--- Send commands to actuators ---*/
         // set the heading
         abstract_robot.setSteerHeading(curr_time, result.t);
   
         // set speed at maximum
         abstract_robot.setSpeed(curr_time, 1.0);
   
         // kick it if we can
         if (abstract_robot.canKick(curr_time))
            if( kick_ball )
            {
System.out.println( "--> Goalie clear the ball!" );
               abstract_robot.kick(curr_time);
            }
      }
      else if( abstract_robot.getPlayerNumber(curr_time)==1 )
      {
         // play forward
         if( (i_am_on_west_team() && (me.x>-0.1)) || 
             (i_am_on_east_team() && (me.x<+0.1)) )
         {
            super.TakeStep();
         }
         else
         {
            result = toward( me, new Vec2( 0, 0 ) );
            kick_ball = true;
   
            /*--- Send commands to actuators ---*/
            // set the heading
            abstract_robot.setSteerHeading(curr_time, result.t);
      
            // set speed at maximum
            abstract_robot.setSpeed(curr_time, 1.0);
      
            // kick it if we can
            if (abstract_robot.canKick(curr_time))
               if( kick_ball )
               {
                  abstract_robot.kick(curr_time);
               }
         }
      }
      else
      {
         super.TakeStep();
      }
      return(CSSTAT_OK);
   }
}
 
