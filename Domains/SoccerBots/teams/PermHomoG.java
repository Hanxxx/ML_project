/**<pre>
===============================================================================
    File: PermHomoG.java
      By: Gita Sukthankar               (gitars@cs.cmu.edu)
===============================================================================
</pre>*/

import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;


public class PermHomoG extends ControlSystemSS {

    public  boolean BehindBall(Vec2 ball, Vec2 opp_goal) {
      if (ball.x*opp_goal.x > 0) {
        return true;
      } else {
        return false;
      }
    }

    public  boolean hasArrived(Vec2 target) {
      if (target.r < 0.5*abstract_robot.RADIUS) {
        return true;
      } else {
        return false;
      }
    }

    public double distance (Vec2 a, Vec2 b) {
      Vec2 t=new Vec2(a);
      t.sub(b);
      return(t.r);
    }

    public double evaluate(int[] perm_row, Vec2[] all_team, Vec2[] all_pos) {
      double sum=0.0;
      for (int i=0; i < perm_row.length; i++) {
        sum+=distance(all_team[i], all_pos[perm_row[i]]);
      }
      return(sum);
    }
    public int optimize(Vec2[] all_team, Vec2[] all_pos) {
      int strategy=0;
      double least_move=9999.0;

      for (int i=0; i < 120; i++) {
        double buf=evaluate(permutation[i], all_team, all_pos);
        if (buf < least_move) {
          strategy=i;
          least_move=buf;
        }
      }
        return(strategy);
    }

    public void Configure() { }
                
        
    public int TakeStep() {
      Vec2      result = new Vec2(0,0);
      long      curr_time = abstract_robot.getTime();


      //retrieve all all the possible sensor information
      Vec2 ball = abstract_robot.getBall(curr_time);
      Vec2 me = abstract_robot.getPosition(curr_time);

      Vec2 ourgoal = abstract_robot.getOurGoal(curr_time);
      Vec2 theirgoal = abstract_robot.getOpponentsGoal(curr_time);

      Vec2[] teammates = abstract_robot.getTeammates(curr_time);
      Vec2[] opponents = abstract_robot.getOpponents(curr_time);

      //heading is given in radians
      double heading=abstract_robot.getSteerHeading(curr_time);

      //compute useful features in the environment
      Vec2 center=new Vec2(ourgoal);
      center.add(theirgoal);
      center.setr(center.r/2);  //scale it by half
      Vec2 center_disp=new Vec2(ourgoal);
      center_disp.sub(theirgoal);
      center_disp.setr(abstract_robot.RADIUS);
      center.add(center_disp);

      boolean behindBall=BehindBall(ball, theirgoal);

      Vec2 defspot=new Vec2(ball.x, ball.y);
      defspot.add(ourgoal);
      defspot.setr(defspot.r/2);

      //determine most dangeorus opponent (opponent closest to ball)
      Vec2 min = new Vec2(99999,0);
      Vec2 minarg = new Vec2();

      for (int i=0; i< opponents.length; i++) {
        Vec2 buf = new Vec2(opponents[i]);
        buf.sub(ball);
        if (buf.r < min.r) {
          min=buf;
          minarg=opponents[i];
        }
      }
      Vec2 blockspot=new Vec2(ball.x, ball.y);
      blockspot.add(minarg);
      blockspot.setr(blockspot.r/2);

      Vec2 closestteammate = new Vec2(99999,0);
      Vec2 closestopponent = new Vec2(99999,0);
      for (int i=0; i< teammates.length; i++) {
        if (teammates[i].r < closestteammate.r) {
          closestteammate = teammates[i];
        }
      }
      for (int i=0; i< opponents.length; i++) {
        if (opponents[i].r < closestopponent.r) {
          closestopponent = opponents[i];
        }
      }


      // compute a point one robot radius behind ball as best kickspot
      Vec2 kickspot = new Vec2(ball.x, ball.y);
      kickspot.sub(theirgoal); //subtract their goal
      kickspot.setr(abstract_robot.RADIUS);
      kickspot.add(ball);
      Vec2 goalie = new Vec2(ourgoal.x, ball.y);

      Vec2 goalie1 = new Vec2(ourgoal.x,
                      ourgoal.y-abstract_robot.RADIUS);
      Vec2 goalie2 = new Vec2(ourgoal.x,
                      ourgoal.y+abstract_robot.RADIUS);

      Vec2 backspot = new Vec2(ball.x, ball.y);
      backspot.sub(theirgoal);
      backspot.setr(abstract_robot.RADIUS*5);
      backspot.add(ball);

      //get setup for iterating through the permutation array
      Vec2[] all_team=new Vec2[5] ;
      Vec2[] all_pos=new Vec2[5] ;
      all_team[0]=new Vec2(0,0);  //myself
      if (teammates.length==0) {
        return(CSSTAT_OK);
      }
      for (int i=0; i< teammates.length; i++) {
        all_team[i+1]=new Vec2(teammates[i]);
      }
      all_pos[0]=new Vec2(kickspot);
      all_pos[1]=new Vec2(defspot);
      all_pos[2]=new Vec2(blockspot);
      all_pos[3]=new Vec2(goalie);
      all_pos[4]=new Vec2(backspot);
      int strategy=optimize(all_team, all_pos);
      result=all_pos[permutation[strategy][0]];


      //only kick if behind the Ball
      if ((behindBall==true) &&
        abstract_robot.canKick(curr_time)) {
        abstract_robot.kick(curr_time);
        abstract_robot.setDisplayString("good position");
      } else if (abstract_robot.canKick(curr_time)) {
        abstract_robot.setDisplayString("can kick");
      } else {
        abstract_robot.setDisplayString("");
      }

      abstract_robot.setDisplayString("" +  permutation[strategy][0]);


      if (hasArrived(result)) {
        abstract_robot.setSpeed(curr_time, 0.0);
        abstract_robot.setSteerHeading(curr_time, theirgoal.t);
      } else {
        boolean isBlocked=false;
        if ((ball.r < 2*abstract_robot.RADIUS) &&
            (Math.abs(ball.t - result.t) < Math.PI/8)) {
          isBlocked=true;
        }
        for (int i=0; i < teammates.length; i++) {
          if (isBlocked==true) {
            break;
          }
          if ((teammates[i].r < 2*abstract_robot.RADIUS) &&
              (Math.abs(teammates[i].t - result.t) < Math.PI/8)) {
            isBlocked=true;
          }
        }
        for (int i=0; i < opponents.length; i++) {
          if (isBlocked==true) {
            break;
          }
          if ((opponents[i].r < 2*abstract_robot.RADIUS) &&
              (Math.abs(opponents[i].t - result.t) < Math.PI/8)) {
            isBlocked=true;
          }
        }

        double offset= isBlocked ? -1*Math.PI/8: 0.0;
        offset=((strategy%2==0) ? offset : -offset);
        abstract_robot.setSteerHeading(curr_time, result.t+offset);
        abstract_robot.setSpeed(curr_time, 1.0);
      }

      return(CSSTAT_OK);
    }

    public static final int permutation[][] = {
      {0, 1, 2, 3, 4},
      {0, 1, 2, 4, 3},
      {0, 1, 3, 2, 4},
      {0, 1, 4, 2, 3},
      {0, 1, 3, 4, 2},
      {0, 1, 4, 3, 2},
      {0, 2, 1, 3, 4},
      {0, 2, 1, 4, 3},
      {0, 3, 1, 2, 4},
      {0, 4, 1, 2, 3},
      {0, 3, 1, 4, 2},
      {0, 4, 1, 3, 2},
      {0, 2, 3, 1, 4},
      {0, 2, 4, 1, 3},
      {0, 3, 2, 1, 4},
      {0, 4, 2, 1, 3},
      {0, 3, 4, 1, 2},
      {0, 4, 3, 1, 2},
      {0, 2, 3, 4, 1},
      {0, 2, 4, 3, 1},
      {0, 3, 2, 4, 1},
      {0, 4, 2, 3, 1},
      {0, 3, 4, 2, 1},
      {0, 4, 3, 2, 1},
      {1, 0, 2, 3, 4},
      {1, 0, 2, 4, 3},
      {1, 0, 3, 2, 4},
      {1, 0, 4, 2, 3},
      {1, 0, 3, 4, 2},
      {1, 0, 4, 3, 2},
      {2, 0, 1, 3, 4},
      {2, 0, 1, 4, 3},
      {3, 0, 1, 2, 4},
      {4, 0, 1, 2, 3},
      {3, 0, 1, 4, 2},
      {4, 0, 1, 3, 2},
      {2, 0, 3, 1, 4},
      {2, 0, 4, 1, 3},
      {3, 0, 2, 1, 4},
      {4, 0, 2, 1, 3},
      {3, 0, 4, 1, 2},
      {4, 0, 3, 1, 2},
      {2, 0, 3, 4, 1},
      {2, 0, 4, 3, 1},
      {3, 0, 2, 4, 1},
      {4, 0, 2, 3, 1},
      {3, 0, 4, 2, 1},
      {4, 0, 3, 2, 1},
      {1, 2, 0, 3, 4},
      {1, 2, 0, 4, 3},
      {1, 3, 0, 2, 4},
      {1, 4, 0, 2, 3},
      {1, 3, 0, 4, 2},
      {1, 4, 0, 3, 2},
      {2, 1, 0, 3, 4},
      {2, 1, 0, 4, 3},
      {3, 1, 0, 2, 4},
      {4, 1, 0, 2, 3},
      {3, 1, 0, 4, 2},
      {4, 1, 0, 3, 2},
      {2, 3, 0, 1, 4},
      {2, 4, 0, 1, 3},
      {3, 2, 0, 1, 4},
      {4, 2, 0, 1, 3},
      {3, 4, 0, 1, 2},
      {4, 3, 0, 1, 2},
      {2, 3, 0, 4, 1},
      {2, 4, 0, 3, 1},
      {3, 2, 0, 4, 1},
      {4, 2, 0, 3, 1},
      {3, 4, 0, 2, 1},
      {4, 3, 0, 2, 1},
      {1, 2, 3, 0, 4},
      {1, 2, 4, 0, 3},
      {1, 3, 2, 0, 4},
      {1, 4, 2, 0, 3},
      {1, 3, 4, 0, 2},
      {1, 4, 3, 0, 2},
      {2, 1, 3, 0, 4},
      {2, 1, 4, 0, 3},
      {3, 1, 2, 0, 4},
      {4, 1, 2, 0, 3},
      {3, 1, 4, 0, 2},
      {4, 1, 3, 0, 2},
      {2, 3, 1, 0, 4},
      {2, 4, 1, 0, 3},
      {3, 2, 1, 0, 4},
      {4, 2, 1, 0, 3},
      {3, 4, 1, 0, 2},
      {4, 3, 1, 0, 2},
      {2, 3, 4, 0, 1},
      {2, 4, 3, 0, 1},
      {3, 2, 4, 0, 1},
      {4, 2, 3, 0, 1},
      {3, 4, 2, 0, 1},
      {4, 3, 2, 0, 1},
      {1, 2, 3, 4, 0},
      {1, 2, 4, 3, 0},
      {1, 3, 2, 4, 0},
      {1, 4, 2, 3, 0},
      {1, 3, 4, 2, 0},
      {1, 4, 3, 2, 0},
      {2, 1, 3, 4, 0},
      {2, 1, 4, 3, 0},
      {3, 1, 2, 4, 0},
      {4, 1, 2, 3, 0},
      {3, 1, 4, 2, 0},
      {4, 1, 3, 2, 0},
      {2, 3, 1, 4, 0},
      {2, 4, 1, 3, 0},
      {3, 2, 1, 4, 0},
      {4, 2, 1, 3, 0},
      {3, 4, 1, 2, 0},
      {4, 3, 1, 2, 0},
      {2, 3, 4, 1, 0},
      {2, 4, 3, 1, 0},
      {3, 2, 4, 1, 0},
      {4, 2, 3, 1, 0},
      {3, 4, 2, 1, 0},
      {4, 3, 2, 1, 0},
    };
}
