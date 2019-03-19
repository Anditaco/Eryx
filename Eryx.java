import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;

import robocode.AdvancedRobot;
import robocode.Condition;
import robocode.HitRobotEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

public class Eryx extends AdvancedRobot{
    static final int VELOCITY_INDEXES = 7;
    static final int RADIAL_INDEXES = 5;
    static final int BINS = 25;
    
    ArrayList<Wave> waves = new ArrayList<Wave>();
    static HashMap<String, int[][][][]> enemyToBuffers = new HashMap<String, int[][][][]>();
    
    boolean ramming = false;
    String target;
    
    double enemyX, enemyY;
    double lastEnemyVelocity = 0;
    double enemyDirection = 1;
    double direction = 1;
    
    long lastScanTime;
    long lastTurnTime;
    
    @Override
    public void run(){
        this.setColors(new Color(79, 17, 57), new Color(0,0,0), new Color(0,0,0));
        this.setScanColor(new Color(15,11,25));
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        lastScanTime = getTime();
        lastTurnTime = getTime();
        while(true) {
            setTurnRadarLeftRadians(Double.POSITIVE_INFINITY);
            waitFor(new ScanDroughtCondition());
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
    	target = e.getName();
    	if(!enemyToBuffers.containsKey(target)) {
    		enemyToBuffers.put(e.getName(), new int[VELOCITY_INDEXES][VELOCITY_INDEXES][RADIAL_INDEXES][BINS]);
    	}
    	
        enemyX = getX() + Math.sin(getHeadingRadians() + e.getBearingRadians())*e.getDistance();
        enemyY = getY() + Math.cos(getHeadingRadians() + e.getBearingRadians())*e.getDistance();
        
        for(int i = 0; i < waves.size(); i++) {
            Wave w = waves.get(i);
            if(w.detectHit(enemyX, enemyY, getTime())) {
                waves.remove(w);
                i--;
            }
        }
        
        enemyDirection = Utilities.sign(e.getVelocity() * Math.sin(e.getHeadingRadians() - (getHeadingRadians() + e.getBearingRadians())));
        
        int[] aimingBins = Utilities.getStats(e.getName(), e.getVelocity(), lastEnemyVelocity, e.getDistance());
        int bestAimingRegister = (BINS-1)/2;
        for(int i = 0; i < BINS; i++)
            if(aimingBins[bestAimingRegister] < aimingBins[i]) bestAimingRegister = i;
        
        double power = aimingBins[bestAimingRegister]!=0 ? 3.0-2.9/(aimingBins[bestAimingRegister]/5) : 0.1;
        Wave aimingWave = new Wave(power, e.getBearingRadians(), enemyDirection, aimingBins);
        
        double gF = (double)(bestAimingRegister - (aimingBins.length-1)/2) / ((aimingBins.length-1)/2);
        double theta = enemyDirection * gF * Utilities.maxEscapeAngle(power);
        double gunAdjust = Utils.normalRelativeAngle(getHeadingRadians() + e.getBearingRadians() - getGunHeadingRadians() + theta);
        setTurnGunRightRadians(gunAdjust);
        
        waves.add(aimingWave);
        if(getGunHeat()==0 && gunAdjust < Math.atan2(9,  e.getDistance()))
            setFireBullet(power);

        if(Math.random()*(getTime()-lastTurnTime) >= 25 && !ramming) {
            direction *= -1;
            lastTurnTime = getTime();
        }
        
        setMaxVelocity(10.0/(180.0/e.getDistance()/Math.PI+.75));
        double approachAngle = Math.asin(e.getVelocity()*Math.sin(getHeadingRadians() + e.getBearingRadians() - Math.PI)/Rules.MAX_VELOCITY) + Math.max(Math.min((getEnergy()-e.getEnergy())/5.0,1),-1)*Math.PI/2;
        if(e.getDistance() > Rules.RADAR_SCAN_RADIUS/2 &&  approachAngle < 0) approachAngle = 0;
        double desiredHeading = (getHeadingRadians()) + e.getBearingRadians() + Math.PI/2 + (Math.PI - approachAngle)*direction;
        
        ramming = approachAngle >= Math.PI/4 ? true : false;

        if(getX() < 50 || getX() > getBattleFieldWidth() - 50 || getY() < 50 || getY() > getBattleFieldHeight() - 50) {
	        if(getX() < 50) {
	            desiredHeading = Math.PI/2;
	        }
	        else if(getX() > getBattleFieldWidth() - 50) {
	            desiredHeading = Math.PI*3/2;
	        }
	        else if(getY() < 50) {
	            desiredHeading = 0;
	        }
	        else if(getY() > getBattleFieldHeight() - 50) {
	            desiredHeading = Math.PI;
	        }
	        
	        if(Math.abs(Utils.normalRelativeAngle(desiredHeading - getHeadingRadians())) > Math.PI/2) {
	        	desiredHeading = (desiredHeading + Math.PI)%(Math.PI*2);
	        	direction = -1;
	        }
	        else {
	        	direction = 1;
	        }
	        setTurnRightRadians(Utils.normalRelativeAngle(desiredHeading - getHeadingRadians()));
        }
        else {
        	setTurnLeftRadians(Utils.normalRelativeAngle(desiredHeading - getHeadingRadians()));
        }
        setAhead(Double.POSITIVE_INFINITY*direction);
        
        lastScanTime = getTime();
        lastEnemyVelocity = e.getVelocity();
        setTurnRadarLeftRadians(1.2*Utils.normalRelativeAngle((Math.atan2(enemyY-getY(), enemyX-getX()) - Utilities.normalize(getRadarHeadingRadians()))));
        scan();
    }
    
    @Override
    public void onHitRobot(HitRobotEvent e) {
    	if(!e.getName().equals(target)) {
    		direction *= -1;
            lastTurnTime = getTime();
    	}
    }
    
    public class Wave{
        private double startX, startY, fireTime, power, startBearing, direction;
        int[] stats;
        
        public Wave(double power, double bearing, double direction, int[] bins) {
            startX = getX();
            startY = getY();
            fireTime = getTime();
            this.power = power;
            startBearing = getHeadingRadians() + bearing;
            this.direction = direction;
            stats = bins;
        }
        
        public boolean detectHit(double eX, double eY, long currentTime) {
            if(Point2D.distance(startX, startY, eX, eY) <= (currentTime-fireTime)*Utilities.getSpeed(power)) {
                double desiredDirection = Math.atan2(eX-startX, eY-startY);
                double angleOffset = Utils.normalRelativeAngle(desiredDirection-startBearing);
                double guessFactor = Math.max(-1, Math.min(1, angleOffset/Utilities.maxEscapeAngle(power)))*direction;
                int index = (int) Math.round((stats.length-1)/2 * (guessFactor+1));
                stats[index]++;
                return true;
            }
            return false;
        }
    }
    
    public class ScanDroughtCondition extends Condition {
        final int DELAY = 5;
        @Override
        public boolean test() {
            if(getTime()-DELAY >= lastScanTime) {
            	ramming = false;
            	return true;
            }
            return false;
        }
    }
    
    public static class Utilities{
        static double normalize(double theta) {
            return (Math.PI*5/2 - theta)%(Math.PI*2);
        }
        
        static int sign(double v) {
            return v < 0 ? -1 : 1;
        }
        
        static double getSpeed(double power) {
            return Rules.getBulletSpeed(power);
        }
        
        static double maxEscapeAngle(double power) {
            return Math.asin(8 / getSpeed(power));
        }
      
        static int[] getStats(String name, double velocity, double lastVelocity, double distance) {
            int velocityIndex = (int)Math.abs(velocity / (Rules.MAX_VELOCITY/(VELOCITY_INDEXES-1)));
            int lastVelocityIndex = (int)Math.abs(lastVelocity / (Rules.MAX_VELOCITY/(VELOCITY_INDEXES-1)));
            int radialIndex = (int)(distance/(Rules.RADAR_SCAN_RADIUS/(double)RADIAL_INDEXES));
            return enemyToBuffers.get(name)[velocityIndex][lastVelocityIndex][radialIndex];
        }
    }
}
