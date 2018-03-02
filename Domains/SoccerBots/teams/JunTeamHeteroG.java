import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.abstractrobot.*;

public class JunTeamHeteroG extends ControlSystemSS {

    //////////////////////////////////////////////////
    // Constants                                    //
    //////////////////////////////////////////////////

    public static final int MEMORY = 10;
    public static final int PLAYERS = 5;
    public static final int TEAMMATES = 4;

    public static final int UNDEFINED = -1;
    public static final int GOALKEEPER = 0;
    public static final int CENTER = 1;
    public static final int TACKLER = 2;
    public static final int DRIBBLER = 3;
    public static final int STOPPER = 4;
    public static final int DEFENDER = 5;

    public static final double INFINITY = 99999;
    public static final double EPSILON = 0.01;
    public static final double ZERO = 0.0;

    public static final double GOALMAX = 0.25;
    public static final double DZYMAX = 0.5;
    public static final double DZYMIN = -0.5;
    public static final double DZXMAX = 1.145;
    public static final double DZXMIN = -1.145;
    public static final double XMAX = 1.3096740;
    public static final double YMAX = 0.6980531;
    public static final double XMIN = -1.3096740;
    public static final double YMIN = -0.6980531;

    public static final int WESTSIDE = -1;
    public static final int EASTSIDE = 1;


    public static final double MAXSPEED = 1.0;
    public static final double HALFSPEED = 0.50;
    public static final double STOP = 0.0;

    //////////////////////////////////////////////////
    // Memory Variables                             //
    //////////////////////////////////////////////////

    private static int goalkeeper;
    private static int center;
    private static int tackler;
    private static int dribbler;
    private static int stopper;
    private static int defender;

    private static Vec2[][] gplayerlocations;
    private static int[][] gplayerpositions;
    private static Vec2 gplayerlastlocation;
    private static int gplayerlastposition;

    private static Vec2[] gballlocations;
    private static Vec2 gballlastlocation;

    private static int side;
    private static String cstrategy;

    //////////////////////////////////////////////////
    // Sensor Data                                  //
    //////////////////////////////////////////////////    

    private long ctime;
    private int mynumber;

    private Vec2 gball, eball;
    private Vec2 gplayer, eplayer;
    private Vec2 gourgoal, eourgoal; 
    private Vec2 goppgoal, eoppgoal;
    
    private Vec2[] gourteam, eourteam;
    private Vec2[] goppteam, eoppteam;

    private Vec2 eopponent, gopponent;
    private Vec2 eteammate, gteammate;

    //////////////////////////////////////////////////
    // Actuator Data                                //
    //////////////////////////////////////////////////    
    
    private boolean attemptkick;
    private Vec2 nextmove;
    private int nextposition;

    /** 
        Configure the control system. This method is
        called once at initialization time. You can 
        use it to do whatever you like.
    */
    public void configure() {
        
        ctime = abstract_robot.getTime();
        cstrategy = "begin";

        goalkeeper = 0;
        center = 0;
        tackler = 0;
        dribbler = 0;
        stopper = 0;
        defender = 0;

        gballlocations = new Vec2[MEMORY];
        for (int i = 0; i < MEMORY; i++) {
            gballlocations[i] = new Vec2(0, 0);
        }
        
        gplayerlocations = new Vec2[PLAYERS][MEMORY];
        gplayerpositions = new int[PLAYERS][MEMORY];
        for (int i = 0; i < PLAYERS; i++) {
            for (int j = 0; j < MEMORY; j++) {
                gplayerlocations[i][j] = new Vec2(0, 0);
                gplayerpositions[i][j] = UNDEFINED;
            }
        }
        
        if (abstract_robot.getOurGoal(ctime).x < 0) 
            side = -1;
        else
            side = 1;

        nextmove = new Vec2(0, 0);
    }

    /**
       Called every timestep to allow the control system to run.
    */
    public int takeStep() {

        updateSensors();
        selectStrategy();
        updateActuators();
       
        return CSSTAT_OK;
    }

    /**
       Updates sensors.
    */

    private void updateSensors() {

        //////////////////////////////////////////////////
        // Sensor Data                                  //
        //////////////////////////////////////////////////    

        ctime = abstract_robot.getTime();
        mynumber = abstract_robot.getPlayerNumber(ctime);

        eplayer = new Vec2(0,0);
        gplayer = abstract_robot.getPosition(ctime);

        eball = abstract_robot.getBall(ctime);
        gball = new Vec2(0,0);
        gball.sett(eball.t);
        gball.setr(eball.r);
        gball.add(gplayer);

        eourgoal = abstract_robot.getOurGoal(ctime);
        gourgoal = new Vec2(0,0);
        gourgoal.sett(eourgoal.t);
        gourgoal.setr(eourgoal.r);
        gourgoal.add(gplayer);

        eoppgoal = abstract_robot.getOpponentsGoal(ctime);
        goppgoal = new Vec2(0,0);
        goppgoal.sett(eoppgoal.t);
        goppgoal.setr(eoppgoal.r);
        goppgoal.add(gplayer);
        
        eourteam = abstract_robot.getTeammates(ctime);
        eoppteam = abstract_robot.getOpponents(ctime);
        
        eteammate = closestTeammate();
        eopponent = closestOpponent();
    }

    /**
       Selects a strategy to execute. Ideally, would select strategy
       based on opponent's history and formation. Didn't have time
       to finish.
    */
   private void selectStrategy() {
       // homogeneous();
       // heterogeneous();
       // kechze();
       // schema();
       // basicteam();
       dteam();
       // Wall();
       // Horde();
   }

    /**
       Homogeneous strategy.
     */
    private void homogeneous() {
        
        cstrategy = "homogeneous";

        if (closestToOppGoal(gplayer))
            playStopper();
        else if (neargoalzone(gball) && closestToGoal(gplayer))
            playGoalkeeper(false);
        else if (neargoalzone(gball) && (goalkeeper > 0) && (defender == 0))
            playDefender();
        else if (nearcenterzone(gball) && closestToGoal(gplayer))
            playGoalkeeper(false);
        else if (nearcenterzone(gball) && (goalkeeper > 0) && (defender == 0))
            playDefender();
        else if (closestToGoal(gplayer))
            playGoalkeeper(false);
        else if (closestToBall(gplayer))
            playDribbler();
        else if (closestToCenter(gplayer) && !closestToBall(gplayer) && (goalkeeper > 0))
            playCenter();
        else
            playTackler();

    }

    private void dteam() {
        
        cstrategy = "dteam";

        if (mynumber == 0)
            playStopper();
        else if (mynumber == 1)
            playCenter();
        else if (mynumber == 2)
            playCenter();
        else if (closestToCenter(gplayer) && !closestToBall(gplayer))
            playCenter();
        else 
            playDribbler();

    }

    private void kechze() {

        cstrategy = "kechze";

        if ((lastPosition() == DRIBBLER) && (!closestToBall(gplayer)) && (dribbler >= 2) && (closestToCenter(gplayer))) {
            playCenter();
        } else if (lastPosition() == GOALKEEPER) {
            playGoalkeeper(false);
        } else if (lastPosition() == STOPPER) {
            playStopper();
        } else if (lastPosition() == DRIBBLER) {
            playDribbler();
        } else if (lastPosition() == TACKLER) {
            playTackler();
        } else if ((goalkeeper < 2) && (defzone(gplayer))) {
            playGoalkeeper(false);
        } else if (stopper == 0) {
            playStopper();
        } else if (tackler == 0) {
            playTackler();
        } else { 
            playCenter();
        }
    }
    
    //////////////////////////////////////////////////
    // Player functions                             //
    //////////////////////////////////////////////////

    /**
       Has player play goalkeeper. Defends our goal.
    */
    private void playGoalkeeper(boolean dontmove) {     
        abstract_robot.setDisplayString("Goalkeeper");
        nextposition = GOALKEEPER;

        if (!closestToGoal(gplayer)) {
            playDribbler();
        } else if (!defzone(gplayer)) {
            nextmove.sett(eourgoal.t);
            nextmove.setr(HALFSPEED);
        } else if ((defzone(gplayer)) && (!goalzone(gplayer)))  {
            nextmove.sett(eourgoal.t);
            nextmove.setr(HALFSPEED);
        } else if (goalzone(gplayer) && !dontmove) {
            nextmove.sety( (eball.y / Math.abs(eball.y)) * 10 );
            nextmove.setr(MAXSPEED);
        } else if (dontmove) {
            nextmove.setr(STOP);
        } else {
            // Should never end up in this state
            System.out.println("ERROR: goalkeeper");
        }

    }

    /**
       Has player play defender. Defends our goal.
    */
    private void playDefender() {       
        abstract_robot.setDisplayString("Defender");
        nextposition = DEFENDER;

        if (!neargoalzone(gplayer)) {
            nextmove.setx(side * DZXMAX);
            nextmove.sety(ZERO);
        } else if (((side == WESTSIDE) && (eball.x > 0)) || ((side == EASTSIDE) && (eball.x < 0))) {
            nextmove.sett(eball.t);
            nextmove.setr(MAXSPEED);
            kickBall();
        } else if (neargoalzone(gplayer)) {
            Vec2 odribbler = closestTo(eoppteam, eball);
            Vec2 ogoal = new Vec2(eourgoal.x, eourgoal.y);
            ogoal.sub(odribbler);
            ogoal.setr(abstract_robot.RADIUS);
            ogoal.add(odribbler);
            
            nextmove.sett(ogoal.t);
            nextmove.setr(MAXSPEED);
            
            if (odribbler != eopponent)
                avoidCollision();
        } else {
            // Should never end up in this state
            System.out.println("ERROR: defender");
        }
    }
    
    /**
       Has player play center. Looks for ball in
       the center of the field.
    */
    private void playCenter() {
        abstract_robot.setDisplayString("Center");
        nextposition = CENTER;

        Vec2 centerfield = new Vec2(0, 0);
        centerfield = abstract_robot.getPosition(ctime);

        if (side == WESTSIDE)
            centerfield.setx(centerfield.x + (4 * abstract_robot.RADIUS));
        else // EASTSIDE
            centerfield.setx(centerfield.x - (4 * abstract_robot.RADIUS));

        centerfield.setr(-centerfield.r);
        
        if (closestToBall(gplayer)) {
            playDribbler();
        } else {
            moveBehind(centerfield, eoppgoal);
            avoidCollision();
        }
    }

    /**
       Has player play tackler. Tries to get in between
       opponent and our goal.
    */
    private void playTackler() {
        abstract_robot.setDisplayString("Tackler");
        nextposition = TACKLER;

        Vec2 odribbler = closestTo(eoppteam, eball);
        Vec2 ogoal = new Vec2(eourgoal.x, eourgoal.y);
        ogoal.sub(odribbler);
        ogoal.setr(abstract_robot.RADIUS);
        ogoal.add(odribbler);

        nextmove.sett(ogoal.t);
        nextmove.setr(MAXSPEED);

        if (odribbler != eopponent)
            avoidCollision();
    }

    /**
       Has player play dribbler. Tries to move the ball
       towards the opponents goal.
    */
    private void playDribbler() {
        abstract_robot.setDisplayString("Dribbler");
        nextposition = DRIBBLER;

        if (behindBall(eball, eoppgoal) && eball.t < abstract_robot.RADIUS * 4) {
            nextmove.sett(eoppgoal.t);
            nextmove.setr(MAXSPEED);
            
            if ((Math.abs(abstract_robot.getSteerHeading(ctime) - eoppgoal.t) < Math.PI / 8) && 
                (eoppgoal.r < abstract_robot.RADIUS * 15) && (!neargoalzone(gplayer) && !nearcenterzone(gplayer))) {
                kickBall();
            }
        } else {
            moveBehind(eball, eoppgoal);
            avoidCollision();
        }
    }

    /**
       Has player play stopper. Tries to prevent opponent
       goalkeeper from doing its job.
     */
    private void playStopper() {
        abstract_robot.setDisplayString("Stopper");
        nextposition = STOPPER;

        // the other team's goalie is whoever is closest to the goal
        Vec2 ogoalkeeper = closestTo(eoppteam, eoppgoal);
        
        eoppgoal.sub(ogoalkeeper);
        eoppgoal.setr(abstract_robot.RADIUS);
        eoppgoal.add(ogoalkeeper);
        
        nextmove.sett(eoppgoal.t);
        nextmove.setr(MAXSPEED);
        
        if (ogoalkeeper != eopponent)
            avoidCollision();
    }

    //////////////////////////////////////////////////
    // Movement functions                           //
    //////////////////////////////////////////////////

    /**
       Has player move behind ball (or other point) with 
       respect to some target point.
     */
    private void moveBehind(Vec2 eball, Vec2 etarget) {
        Vec2 behindpoint = new Vec2(0, 0);
        double behind = 0;
        double pointside = 0;
        
        behindpoint.sett(etarget.t);
        behindpoint.setr(etarget.r);
        
        behindpoint.sub(eball);
        behindpoint.setr(-abstract_robot.RADIUS * 1.8);
        
        behind = Math.cos(Math.abs(eball.t - behindpoint.t));
        
        pointside = Math.sin(Math.abs(eball.t - behindpoint.t));
        
        // if you are in FRONT
        if(behind > 0) {
            if( pointside > 0)
                behindpoint.sett(behindpoint.t + Math.PI / 2);
            else
                behindpoint.sett(behindpoint.t - Math.PI / 2);
        }
        
        nextmove.sett(eball.t);
        nextmove.setr(eball.r);
        nextmove.add(behindpoint);
        nextmove.setr(MAXSPEED);
    }

    /**
       If player is approximately behind the ball with
       respect to some target point, returns true.
     */
    private boolean behindBall(Vec2 eball, Vec2 etarget) {
        if (Math.abs(eball.t - etarget.t) < Math.PI / 10) 
            return true;
        else
            return false;
    }

    /**
       Has player kick the ball.
    */
    private void kickBall() {
        attemptkick = true;
    }

    /**
       Has player avoid a collision.
     */
    private void avoidCollision() {
        if (eteammate.r < abstract_robot.RADIUS * 1.4) {
            nextmove.setx(-eteammate.x);
            nextmove.sety(-eteammate.y);
            nextmove.setr(MAXSPEED);
        } else if (eopponent.r < abstract_robot.RADIUS * 1.4) {
            nextmove.setx(-eopponent.x);
            nextmove.sety(-eopponent.y);
            nextmove.setr(MAXSPEED);
        }
    }

    //////////////////////////////////////////////////
    // Proximity functions                          //
    //////////////////////////////////////////////////

    /**
       Returns true if player is closest to ball.
     */
    private boolean closestToBall(Vec2 gplayer) {
        return closestTo(gplayer, gball);
    }
    
    /**
       Returns true if player is closest to our goal.
    */
    private boolean closestToGoal(Vec2 gplayer) {
        return closestTo(gplayer, gourgoal);
    }
    
    /**
       Returns true if player is closest to opponents goal.
    */

    private boolean closestToOppGoal(Vec2 gplayer) {
        return closestTo(gplayer, goppgoal);
    }

    /** 
        Returns true if player is closest to center of field.
     */
    private boolean closestToCenter(Vec2 gplayer) {
        return closestTo(gplayer, new Vec2(ZERO, ZERO));
    }

    /**
       Returns true if player is closest to opponent goalkeeper.
     */
    private boolean closestToGoalkeeper(Vec2 gplayer) {

        return false;
    }

    /**
       Returns true if player is closest to opponent dribbler.
     */
    private boolean closestToDribbler(Vec2 gplayer) {

        return false;
    }

    /**
       Returns true if player is closest to target point among
       teammates.
    */
    private boolean closestTo(Vec2 gplayer, Vec2 gtarget) {

        Vec2 temp = new Vec2(gplayer.x, gplayer.y);
        temp.sub(gtarget);
        
        double MyDist = temp.r;
        for (int i = 0; i < eourteam.length; i++)
            {
                temp = new Vec2(eourteam[i].x, eourteam[i].y);
                temp.add(gplayer);
                temp.sub(gtarget);
                double TheirDist = temp.r;
                if (TheirDist < MyDist)
                    return false;
            }
        return true;
    }

    /**
       Returns vector towards the closest opponent.
     */
    private Vec2 closestOpponent() {
        
        Vec2 closest = new Vec2(INFINITY, 0);
        for (int i = 0; i < eoppteam.length; i++) {
            if (eoppteam[i].r < closest.r)
                closest = eoppteam[i];
        }
        return closest;
    }
    
    /**
       Returns the vector towards the closest teammate.
    */
    private Vec2 closestTeammate() {
        Vec2 closest = new Vec2(INFINITY, 0);
        for (int i = 0; i < eourteam.length; i++) {
            if (eourteam[i].r < closest.r)
                closest = eourteam[i];
        }
        return closest;
    }

    /**
       Returns player among players closest to target.
     */

    private Vec2 closestTo(Vec2[] objects, Vec2 point) {
        double dist = INFINITY;
        Vec2 result = new Vec2(0, 0);
        Vec2 temp = new Vec2(0, 0);
        
        for( int i=0; i < objects.length; i++)
            {
                
                // find the distance from the point to the current
                // object
                temp.sett( objects[i].t);
                temp.setr( objects[i].r);
                temp.sub( point);
                
                // if the distance is smaller than any other distance
                // then you have something closer to the point
                if(temp.r < dist)
                    {
                        result = objects[i];
                        dist = temp.r;
                    }
            }
        
        return result;
    }
    
    //////////////////////////////////////////////////
    // Zone functions                               //
    //////////////////////////////////////////////////
    
    /**
       If point is in goalzone, returns true.
    */
    private boolean goalzone(Vec2 gpoint) {
        if (side == WESTSIDE) {
            if ( (Math.abs(XMIN - gpoint.x) < EPSILON) && (Math.abs(gpoint.y) < GOALMAX) ) {
                return true;
            }
        } else {
            if ( (Math.abs(XMAX - gpoint.x) < EPSILON) && (Math.abs(gpoint.y) < GOALMAX) ) {
                return true;
            }
        }
        return false;
    }

    /** 
        If point is in defzone, returns true.
    */
    private boolean defzone(Vec2 gpoint) {
        if (side == WESTSIDE) {
            if ((gpoint.x < DZXMIN) && (Math.abs(gpoint.y) < DZYMAX)) {
                return true;
            }
        } else {
            if ((DZXMAX < gpoint.x) && (Math.abs(gpoint.y) < DZYMAX)) {
                return true;
            }
        }       
        return false;
    }

    /** 
        If point is in centerzone, returns true.
    */
    private boolean centerzone(Vec2 gpoint) {
        if (gpoint.r < (abstract_robot.RADIUS * 2)) {
            return true;
        }
        return false;
    }

    /**
       If point is in neargoalzone, returns true.
     */
    private boolean neargoalzone(Vec2 gpoint) {
        if (side == WESTSIDE) {
            if (gpoint.x < (XMIN / 2)) {
                return true;
            }
        } else {
            if ((XMAX / 2) < gpoint.x) {
                return true;
            }
        }       
        return false;
    }

    /**
       If point is in nearcenterzone, returns true.
     */
    private boolean nearcenterzone(Vec2 gpoint) {
        if (side == WESTSIDE) {
            if ( ((XMIN / 2) < gpoint.x) && (gpoint.x < ZERO) ) {
                return true;
            }
        } else {
            if ( (ZERO < gpoint.x) && ((XMAX / 2) < gpoint.x) ) {
                return true;
            }
        }       
        return false;
    }

    /**
       If point is in farcenterzone, returns true.
     */
    private boolean farcenterzone(Vec2 gpoint) {
        Vec2 temp = new Vec2(-gpoint.x, -gpoint.y);
        if (nearcenterzone(temp)) {
            return true; 
        }
        return false;
    }

    /**
       If point is in fargoalzone, returns true.
     */
    private boolean fargoalzone(Vec2 gpoint) {
        Vec2 temp = new Vec2(-gpoint.x, -gpoint.y);
        if (neargoalzone(temp)) {
            return true;
        }
        return false;
    }

    //////////////////////////////////////////////////
    // Memory functions                             //
    //////////////////////////////////////////////////

    /**
       Records player location into memory
     */
    private void recordLocation() {
        for (int i = 0; i < (MEMORY - 1); i++) {
            gplayerlocations[mynumber][i].sett(gplayerlocations[mynumber][i + 1].t);
            gplayerlocations[mynumber][i].setr(gplayerlocations[mynumber][i + 1].r);
        }
        gplayerlocations[mynumber][MEMORY - 1].sett(gplayer.t);
        gplayerlocations[mynumber][MEMORY - 1].setr(gplayer.r);
    }

    /**
       Records player position into memory
    */
    private void recordPosition(int gposition) {
        for (int i = 0; i < (MEMORY - 1); i++) {
            gplayerpositions[mynumber][i] = gplayerpositions[mynumber][i + 1];
        }
        gplayerpositions[mynumber][MEMORY - 1] = gposition;
    }

    /**
       Records ball location into memory
    */
    private void recordBall() {
        for (int i = 0; i < (MEMORY - 1); i++) {
            gballlocations[i] = gballlocations[i + 1];
        }
        gballlocations[MEMORY - 1] = gball;
    }

    /**
       Retrieve last position played
    */
    private int lastPosition() {
        return gplayerpositions[mynumber][MEMORY - 1];
    }

    /**
       Print last position played
     */
    private void displayLastPosition() {
        System.out.print("Player " + mynumber + ": ");
        switch (gplayerpositions[mynumber][MEMORY - 1]) {
        case UNDEFINED:
            System.out.println("undefined");
            break;
        case GOALKEEPER:
            System.out.println("goalkeeper");
            break;
        case CENTER:
            System.out.println("center");
            break;
        case TACKLER:
            System.out.println("tackler");
            break;
        case DRIBBLER:
            System.out.println("dribbler");
            break;
        case STOPPER:
            System.out.println("stopper");
            break;
        case DEFENDER:
            System.out.println("defender");
            break;
        }
    }

    /*
      Update player positions currently filled
    */
    private void updatePositions(int position) {

        boolean different = false;

        switch (position) {
        case GOALKEEPER:
            if (lastPosition() != GOALKEEPER) {
                goalkeeper++;
                different = true;
            }
            break;
        case CENTER:
            if (lastPosition() != CENTER) {
                center++;
                different = true;
            }
            break;
        case TACKLER:
            if (lastPosition() != TACKLER) {
                tackler++;
                different = true;
            }
            break;
        case DRIBBLER:
            if (lastPosition() != DRIBBLER) {
                dribbler++;
                different = true;
            }
            break;
        case STOPPER:
            if (lastPosition() != STOPPER) {
                stopper++;
                different = true;
            }
            break;
        case DEFENDER:
            if (lastPosition() != DEFENDER) {
                defender++;
                different = true;
            }
            break;
        }

        if (different) {
            switch (lastPosition()) {
            case GOALKEEPER:
                goalkeeper--;
                break;
            case CENTER:
                center--;
                break;
            case TACKLER:
                tackler--;
                break;
            case DRIBBLER:
                dribbler--;
                break;
            case STOPPER:
                stopper--;
                break;
            case DEFENDER:
                defender--;
                break;
            case UNDEFINED:
                break;
            }
        }
    }

    //////////////////////////////////////////////////
    // Actuator functions                           //
    //////////////////////////////////////////////////    

    /**
       Executes moves for player
    */
    private void updateActuators() {
        abstract_robot.setSteerHeading(ctime, nextmove.t);
        abstract_robot.setSpeed(ctime, nextmove.r);

        if (attemptkick) {
            if (abstract_robot.canKick(ctime))
                abstract_robot.kick(ctime);
        }
        recordLocation();
        recordBall();
        updatePositions(nextposition);
        recordPosition(nextposition);
    }
}





