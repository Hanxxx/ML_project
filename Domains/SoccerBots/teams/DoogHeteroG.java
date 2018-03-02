/*
 * DoogHeteroG.java
 */

import  EDU.gatech.cc.is.util.Vec2;
import  EDU.gatech.cc.is.abstractrobot.*;
//Clay not used

/**
 * Doog's fubar strategy.
 * 
 * 
 */


public class DoogHeteroG extends ControlSystemSS
{
    // Current state variables
    Vec2 CurMyPos, CurBallPos, CurBallPosEgo;
    long CurTime;
    int CurMode;
    boolean KickIt = false;
    int MyNum;
    Vec2 CurTeammates[], CurOpponents[];
    Vec2 CurOurGoal, CurOpponentsGoal;
    int StuckCount = 0;

    // Previous step state variables
    Vec2 PrevMyPos, PrevBallPos, PrevBallPosEgo;
    long PrevTime;
    int PrevMode;
    Vec2 PrevTeammates[], PrevOpponents[];
  
    // Modes
    final int MODE_ATTACK = 0;
    final int MODE_GOON = 1;
    final int MODE_GOALIE = 2;
    final int MODE_TOCCHET = 3;

    // Useful constants
    final double PI = Math.PI;
    final double DEFAULT_SPEED = 1.0;
    final double DEFENDED_DISTANCE = 0.03;
    final double BETWEEN_GOAL_ANGLE = PI/6;
    final double ROBOT_RADIUS = 0.06;
    final double BALL_RADIUS = 0.02;
    final double GOAL_WIDTH = .4;
    final double STUCK_LIMIT = 100;
    final double KICK_DISTANCE = 1.0;
    /**
    Configure the Avoid control system.  This method is
    called once at initialization time.  You can use it
    to do whatever you like.
    */
    public void Configure()
    {
        PrevMyPos = new Vec2(0,0);
        PrevBallPos = new Vec2(0,0);
        PrevBallPosEgo = new Vec2(0,0);
    }
  
  
    /**
    Called every timestep to allow the control system to
    run.
    */
    public int TakeStep()
    {
        double RandomHeading = 0;
        // This var will hold final steering command at end.
        Vec2 Cmd = new Vec2(0,DEFAULT_SPEED);

    // My number, for debugging purposes only!
        MyNum = abstract_robot.getPlayerNumber(CurTime);
    
        // Read current time
        CurTime = abstract_robot.getTime();
        // Get current position
        CurMyPos = abstract_robot.getPosition(CurTime);
        // Get egocentric ball position, then convert to absolute position
        CurBallPosEgo = abstract_robot.getBall(CurTime);
        CurBallPos = new Vec2(CurBallPosEgo.x, CurBallPosEgo.y);
        CurBallPos.add(CurMyPos);
        // Get goal positions
        CurOurGoal = abstract_robot.getOurGoal(CurTime);
        CurOpponentsGoal = abstract_robot.getOpponentsGoal(CurTime);
        // Get team positions
        CurTeammates = abstract_robot.getTeammates(CurTime);
        CurOpponents = abstract_robot.getOpponents(CurTime);
    
        // Determine my current mode
        //      if(MyNum == 0) CurMode = MODE_GOALIE;
        //      if(MyNum == 1) CurMode = MODE_GOON;
        //      if(MyNum == 2) CurMode = MODE_GOON;
        //      if(MyNum == 3) CurMode = MODE_ATTACK;
        if(MyNum == 4) CurMode = MODE_TOCCHET;
        else if(ShouldAttack()) CurMode = MODE_ATTACK;
        else if(ShouldBeGoalie()) CurMode = MODE_GOALIE;
        else CurMode = MODE_GOON;
    
        // Act based on current mode
        switch(CurMode)
            {
            case(MODE_ATTACK):
                // Go for the glory!
                Cmd = AttackMode();
                break;
      
            case(MODE_GOON):
                //System.out.println("In goon mode.");
                // Find the nearest undefended opponent, and kick his ass!
                Cmd = GoonMode();

                break;
            case(MODE_GOALIE):
                // For now, use stolen goalie function from Kechze.
                Cmd = GoalieMode();
                break;
    
            case(MODE_TOCCHET):
                // Crash the cage
                Cmd = TocchetMode();
                break;
  
            default:
                System.out.println("Unknown Mode!");
                break;
            }

        // Use Cmd!
        if((Math.abs(CurMyPos.x - PrevMyPos.x) == 0.0) &&
           (Math.abs(CurMyPos.y - PrevMyPos.y) == 0.0)) {
                StuckCount++;
            }
        else {
                StuckCount = 0;
            }

        if(StuckCount > STUCK_LIMIT) {
            RandomHeading = Math.random() * 2 * PI;
            abstract_robot.setSteerHeading(CurTime, RandomHeading);
            if(StuckCount > 2 * STUCK_LIMIT) StuckCount = 0;
        }
        else abstract_robot.setSteerHeading(CurTime,Cmd.t);
        abstract_robot.setSpeed(CurTime,1.0);

        // Save old values of everything
        PrevTime = CurTime;
        PrevMyPos = new Vec2(CurMyPos.x, CurMyPos.y);
        PrevBallPos = new Vec2(CurBallPos.x, CurBallPos.y);
        
        // Kick, if we can and wish to
        if(abstract_robot.canKick(CurTime) && KickIt)
            {
                abstract_robot.kick(CurTime);
            }

        // tell the parent we're OK
        return(CSSTAT_OK);
    }

    boolean ShouldBeGoalie()
    {
        if(ClosestTo(CurMyPos, EgoToAbs(CurOurGoal))) return(true);
        return(false);
    }

    boolean ShouldAttack()
    {
        if(ClosestTo(CurMyPos, CurBallPos))
            {
                return(true);
            }
        else
            {
                return(false);
            }
    }

    Vec2 AttackMode()
    {
        abstract_robot.setDisplayString("Attack");
        Vec2 TargetSpot = new Vec2(CurBallPosEgo.x, CurBallPosEgo.y);
        Vec2 GoalSpot = new Vec2(CurOpponentsGoal.x, CurOpponentsGoal.y);
        if(CurMyPos.y > 0) GoalSpot.y += 0.9 * (GOAL_WIDTH / 2.0);
        if(CurMyPos.y < 0) GoalSpot.y -= 0.9 * (GOAL_WIDTH / 2.0);
        TargetSpot.sub(GoalSpot);
        TargetSpot.setr(ROBOT_RADIUS);
        TargetSpot.add(CurBallPosEgo);

        if(Math.abs(CurOpponentsGoal.r) < KICK_DISTANCE) KickIt = true;
        else KickIt = false;

        return(TargetSpot);
    }

    Vec2 GoonMode()
    {
        abstract_robot.setDisplayString("Goon");
        Vec2 Victim = new Vec2(99999,0);
        Vec2 CmdReturn;
        for(int i=0; i < CurOpponents.length; i++)
            {
                if(Undefended(CurOpponents[i]) && (CurOpponents[i].r < Victim.r))
                    Victim = CurOpponents[i];
            }
        return(Victim);
    }
    
    Vec2 TocchetMode()
    {
        abstract_robot.setDisplayString("Tocchet");
        Vec2 Goalie = new Vec2(99999, 0);
        Vec2 GoalDistance = new Vec2(99999,0);
        Vec2 CmdReturn;
        for(int i = 0; i < CurOpponents.length; i++)
            {
                if(ClosestTo(CurOpponents[i], EgoToAbs(CurOpponentsGoal))) {
                    Goalie.setx(CurOpponents[i].x);
                    Goalie.sety(CurOpponents[i].y);
                }
                
            }
        return(Goalie);
    }

    Vec2 GoalieMode()
    {
        abstract_robot.setDisplayString("Goalie");
        Vec2 ReturnCmd = new Vec2(CurOurGoal.x, CurOurGoal.y);
        // If we're too far out of goal in x dir, get back in!
        Vec2 OurGoalAbs = new Vec2(CurOurGoal.x, CurOurGoal.y);
        OurGoalAbs.add(CurMyPos);
        if(Math.abs(CurMyPos.x) < Math.abs(OurGoalAbs.x * 0.9))
            {
                return(CurOurGoal);
            }

        // Otherwise, calculate projected ball trajectory
        Vec2 BallDir = new Vec2(CurBallPos.x, CurBallPos.y);
        BallDir.sub(PrevBallPos);
        // If ball is headed into goal, block it!
        ReturnCmd.setx(0);
    
        boolean MoveUp = false;
        boolean MoveDown = false;

        if(CurMyPos.y < CurBallPos.y) MoveUp = true;
        if(CurMyPos.y > CurBallPos.y) MoveDown = true;
        if(CurMyPos.y > GOAL_WIDTH/2.0) MoveUp = false;
        if(CurMyPos.y < -GOAL_WIDTH/2.0) MoveDown = false;

        if(MoveDown && MoveUp)
            {
                ReturnCmd.sety(0);
                //      System.out.println("Both " + CurMyPos.y + " " + CurBallPos.y);
            }
        else if(MoveDown) 
            {
                ReturnCmd.sety(-1);
                //      System.out.println("Down");
            }
        else if(MoveUp) 
            {
                ReturnCmd.sety(1);
                //      System.out.println("Up");
            }
        else 
            {
                ReturnCmd.sety(0);
            }

        return(ReturnCmd);
    }

    boolean Undefended(Vec2 opponent)
    {
        Vec2 AbsOpp = EgoToAbs(opponent);
        // return true if there is no teammate within DEFENDED_DISTANCE of opponent.
        for(int i = 0; i < CurTeammates.length; i++)
            {
                Vec2 AbsPos = EgoToAbs(CurTeammates[i]);
                Vec2 DiffPos = new Vec2(AbsOpp.x - AbsPos.x, AbsOpp.y - AbsPos.y);
      
                if(DiffPos.r < 2 * ROBOT_RADIUS + DEFENDED_DISTANCE) return(false);
            }
        return(true);
    }

    Vec2 EgoToAbs(Vec2 EgoPos)
    {
        Vec2 AbsPosition = new Vec2(EgoPos.x, EgoPos.y);
        AbsPosition.add(CurMyPos);
        return(AbsPosition);
    }

    boolean DefendingEast()
    {
        return(CurOurGoal.x < 0);
    } 

    boolean BetweenGoal(Vec2 Opp, Vec2 Goal)
    {
        Vec2 OpptoGoal = new Vec2(Goal.x, Goal.y);
        OpptoGoal.sub(Opp);

        Vec2 OpptoMe = new Vec2(CurMyPos.x, CurMyPos.y);
        OpptoMe.sub(Opp);

        if(Math.abs(OpptoGoal.t - OpptoMe.t) < BETWEEN_GOAL_ANGLE)
            {
                return(true);
            }
        return(false);
    }

    boolean BallInOwnZone()
    {
        if(CurOurGoal.x * CurBallPos.x > 0) return(true);
        return(false);
    }

    boolean ClosestTo(Vec2 Me, Vec2 SpotAbs)
    {
        // Stolen from Kechze
        Vec2 temp = new Vec2( Me.x, Me.y);
        temp.sub(SpotAbs);

        double MyDist = temp.r;
        for (int i=0; i< CurTeammates.length; i++)
            {
                temp = new Vec2( CurTeammates[i].x, CurTeammates[i].y);
                temp.add(Me);
                temp.sub(SpotAbs);
                double TheirDist = temp.r;
                if (TheirDist <= MyDist)
                    return false;
            }
        return true;
    }

  
}


