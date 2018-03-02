import	EDU.gatech.cc.is.util.Vec2;
import	EDU.gatech.cc.is.abstractrobot.*;

public class DaveHeteroG extends ControlSystemSS
{
  private double KICK_ANGLE_TOLERANCE = .4;

  /**
     Configure the control system.  This method is
     called once at initialization time.  You can use it
     to do whatever you like.
  */
  public void Configure()
    {
      abstract_robot.setBaseSpeed(abstract_robot.MAX_TRANSLATION);
    }
		
	
  /**
     Called every timestep to allow the control system to
     run.
  */
  public int TakeStep()
    {
      // things to calculate:
      // - Who on my team is closest to the ball
      // - Who on their team is closest to the ball
      // - match up each of my remaining guys to the closest of the
      //   opponent's remaining guys

      // get the current time for timestamps
      long	curr_time = abstract_robot.getTime();

      Vec2 me_abs = abstract_robot.getPosition(curr_time);

      Vec2 ball_abs = abstract_robot.getBall(curr_time);
      ball_abs.add(me_abs);

      Vec2 goal_abs = abstract_robot.getOurGoal(curr_time);
      goal_abs.add(me_abs);

      double goal_dir = (abstract_robot.getOpponentsGoal(curr_time).x > 0) ?
        1 : -1;

      Vec2[] teammates = abstract_robot.getTeammates(curr_time);
      Vec2[] teammates_abs = new Vec2[teammates.length];
      for(int i = 0; i < teammates_abs.length; i++) {
        teammates_abs[i] = new Vec2(teammates[i]);
        teammates_abs[i].add(me_abs);
      }

      Vec2[] opponents = abstract_robot.getOpponents(curr_time);
      Vec2[] opponents_abs = new Vec2[opponents.length];
      for(int i = 0; i < opponents_abs.length; i++) {
        opponents_abs[i] = new Vec2(opponents[i]);
        opponents_abs[i].add(me_abs);
      }

      // find who on our team is closest to the ball
      Vec2 min_us_vec_to_ball = vec_diff(ball_abs, me_abs);
      double my_dist = min_us_vec_to_ball.r;
      int closest_teammate_to_ball = -1; // -1 meaning me
      for(int i = 0; i < teammates_abs.length; i++) {
        Vec2 vec_to_ball =  vec_diff(ball_abs, teammates_abs[i]);
        if(vec_to_ball.r < min_us_vec_to_ball.r) {
          min_us_vec_to_ball = vec_to_ball;
          closest_teammate_to_ball = i;
        }
      }

      // handle offender
      if(abstract_robot.getPlayerNumber(-1) == 3) {
        Vec2 opponents_goal_abs = abstract_robot.getOpponentsGoal(curr_time);
        opponents_goal_abs.add(me_abs);
        
        // find the goal in terms of the ball
        Vec2 dest = vec_diff(opponents_goal_abs, ball_abs);

        double goal_kick_angle = dest.t + 3.141 / 4;

        // find the robot destination, in terms of the ball: (i.e. get
        // behind the ball, facing towards the goal.)
        dest.setr(-abstract_robot.RADIUS * .9
                  /* - abstract_robot.KICKER_SPOT_RADIUS / 2*/);

        // find destination in terms of field:
        dest.add(ball_abs);

        // find destination in terms of me:
        dest = vec_diff(dest, me_abs);

        if(dest.x * goal_dir < 0) {
          // we are behind the ball facing our goal, so go around the
          // ball a bit.
          if(dest.y < 0)
            dest.sety(dest.y + abstract_robot.RADIUS);
          else
            dest.sety(dest.y - abstract_robot.RADIUS);
        }

        // if we are close enough to the ball, turn to kick it.
        if(dest.r < abstract_robot.KICKER_SPOT_RADIUS / 2) {

          if(abstract_robot.canKick(curr_time)) {
            abstract_robot.setDisplayString("kick");

            abstract_robot.kick(curr_time);
          }
          else {
            abstract_robot.setDisplayString("turn to kick");
          
            abstract_robot.setSteerHeading(curr_time, goal_kick_angle);
            abstract_robot.setSpeed(curr_time, 0);
          }
        }
        else {
          abstract_robot.setDisplayString("go to, d=" + dispdub(my_dist));

          abstract_robot.setSteerHeading(curr_time, dest.t);
          if(dest.r < abstract_robot.RADIUS &&
             (me_abs.x - goal_abs.x) * goal_dir < abstract_robot.RADIUS * 2) {
            abstract_robot.setSpeed(curr_time, dest.r / abstract_robot.RADIUS);
          }
          else {
            abstract_robot.setSpeed(curr_time, 1.0);
          }
        }

        return(CSSTAT_OK);
      }
      else { // else we are not the offender

        // us_taken records which teammates are allocated already
        boolean[] us_taken = new boolean[opponents_abs.length];
        for(int i = 0; i < teammates_abs.length; i++)
          us_taken[i] = false;
        int num_us_taken = 0;

        // assume the offender is the closest to the ball.
        if(closest_teammate_to_ball != -1) {
          us_taken[closest_teammate_to_ball] = true;
          num_us_taken++;
        }

        // find who on our team is closest to the goal
        Vec2 min_us_vec_to_goal = abstract_robot.getOurGoal(curr_time);
        int closest_teammate_to_goal = -1; // -1 meaning me
        for(int i = 0; i < teammates_abs.length; i++) {
          Vec2 vec_to_goal = vec_diff(goal_abs, teammates_abs[i]);
          if(vec_to_goal.r < min_us_vec_to_goal.r) {
            min_us_vec_to_goal = vec_to_goal;
            closest_teammate_to_goal = i;
          }
        }

        if(abstract_robot.getPlayerNumber(-1) == 0) { // I am the goalie.
          Vec2 target_pos = new Vec2(0, 0);
          if(goal_abs.x < 0)
            target_pos.setx(goal_abs.x + abstract_robot.RADIUS);
          else
            target_pos.setx(goal_abs.x - abstract_robot.RADIUS);

          double goal_y = Math.min(Math.max(ball_abs.y, -.27), .27);
          target_pos.sety(goal_y);
          
          target_pos.sub(me_abs);

          abstract_robot.setDisplayString("goalie-ing " + dispdub(goal_y));
          
          abstract_robot.setSteerHeading(curr_time, target_pos.t);
          abstract_robot.setSpeed(curr_time, 1.0);

          return(CSSTAT_OK);
        }
        else {
          // I am not the goalie.  Assume the closest robot to the
          // goal is the goalie.
          if(closest_teammate_to_goal != -1) {
            us_taken[closest_teammate_to_goal] = true;
            num_us_taken++;
          }
        }

        ////////////////////////////////////
        // allocate defenders to opponents

        boolean[] them_handled = new boolean[opponents_abs.length];
        for(int i = 0; i < opponents_abs.length; i++)
          them_handled[i] = false;
        int num_them_handled = 0;

        while(num_them_handled < opponents_abs.length &&
              num_us_taken < teammates_abs.length + 1) {
          
          // find who on their team is next closest to the ball
          Vec2 min_them_vec_to_ball = new Vec2(100000, 0);
          int closest_opponent_to_ball = -1;
          for(int i = 0; i < opponents_abs.length; i++) {
            if(!them_handled[i]) {
              Vec2 vec_to_ball =  vec_diff(ball_abs, opponents_abs[i]);
              if(vec_to_ball.r < min_them_vec_to_ball.r) {
                min_them_vec_to_ball = vec_to_ball;
                closest_opponent_to_ball = i;
              }
            }
          }
          if(closest_opponent_to_ball != -1) {
            them_handled[closest_opponent_to_ball] = true;
            num_them_handled++;
          }
          else {
            System.out.println("ERROR! with " + num_them_handled +
                               " opponents handled," +
                               " none are closest to the ball.");
          }

          Vec2 min_us_vec_to_them =
            vec_diff(opponents_abs[closest_opponent_to_ball], me_abs);
          int closest_us_to_them = -1;
          for(int i = 0; i < teammates_abs.length; i++) {
            if(!us_taken[i]) {
              Vec2 vec_to_them =
                vec_diff(opponents_abs[closest_opponent_to_ball],
                         teammates_abs[i]);
              if(vec_to_them.r < min_us_vec_to_them.r) {
                min_us_vec_to_them = vec_to_them;
                closest_us_to_them = i;
              }
            }
          }
          
          if(closest_us_to_them == -1) {
            // I am closest to this opponent
            
            Vec2 goal_pos = vec_diff(opponents_abs[closest_opponent_to_ball],
                                     me_abs);
            min_them_vec_to_ball.setr(abstract_robot.RADIUS * 1.9);
            goal_pos.add(min_them_vec_to_ball);

            // calculate a force pushing this player away from its
            // teammates.
            Vec2 force_sum = new Vec2(0, 0);
            for(int i = 0; i < teammates_abs.length; i++) {
              Vec2 force = new Vec2(teammates[i]);
              force.setr(-abstract_robot.RADIUS * 3 / force.r);
              force_sum.add(force);
            }
            // add in a force pushing this player away from its
            // opponents.
            for(int i = 0; i < opponents_abs.length; i++) {
              Vec2 force = new Vec2(opponents[i]);
              force.setr(-abstract_robot.RADIUS / force.r);
              force_sum.add(force);
            }
            force_sum.setr(force_sum.r * abstract_robot.RADIUS * 2);

            goal_pos.add(force_sum);

            abstract_robot.setDisplayString("blocking");

            abstract_robot.setSteerHeading(curr_time, goal_pos.t);
            abstract_robot.setSpeed(curr_time, 1.0);

            return(CSSTAT_OK);
          }
          else {
            us_taken[closest_us_to_them] = true;
            num_us_taken++;
          }
        }
      }

      // No robot should get to here in the code!  So wander around.

      abstract_robot.setDisplayString("unemployed");

      abstract_robot.setSteerHeading(curr_time, Math.random() * 2 * 3.141);
      abstract_robot.setSpeed(curr_time, 1.0);

      return(CSSTAT_OK);
    }

  // return the vector (to) - (from), i.e. (to) in terms of (from)
  Vec2 vec_diff(Vec2 to, Vec2 from)
    {
      return new Vec2(to.x - from.x, to.y - from.y);
    }

  int dispdub(double d)
    {
      return (int) (d * 1000);
    }

}

