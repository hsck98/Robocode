package u1622228;
import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.util.Hashtable;
import java.util.Enumeration;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;

public class hsck98 extends Robot
{
	static Hashtable<String, Enemy> enemies = new Hashtable<String, Enemy>();
	static Enemy enemy;
	static Point2D.Double nextDestination;
	static Point2D.Double lastPosition;
	static Point2D.Double myPos;
	static double myEnergy;
	static double opponentEnergy;
	int movementDirection = 1;
	double firePower;
	double speedOfBullet;
	double distanceToEnemy;
	RobotStatus robotStatus;

	//---------------------------------- Run ----------------------------------
	//-------------------------------------------------------------------------
	public void run()
	{
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		enemy = new Enemy();
		setColors(Color.pink, Color.pink, Color.pink);
		nextDestination = lastPosition = myPos = new Point2D.Double(robotStatus.getX(), robotStatus.getY());

		while(true) {
			myPos = new Point2D.Double(getX(), getY());
			myEnergy = getEnergy();
			turnRadarRight(360);
			// wait until you have scanned all other bots. this should take around 7 to 9 ticks.
			if(enemy.getLive() && getTime() > 9) {
				if (getOthers() > 1) {
					doMove();
				}
			}
		}
	}

	//---------------------------------- onScannedRobot event ----------------------------------
	//------------------------------------------------------------------------------------------
	public void onScannedRobot(ScannedRobotEvent e) {
		int roundNumber = getRoundNum() + 1;
		Enemy villain;
		if (enemies.containsKey(e.getName())) {
			villain = (Enemy)enemies.get(e.getName());
		} else {
			villain = new Enemy();
			enemies.put(e.getName(), villain);
		}

		villain.setPos(randomPoint(myPos, e.getDistance(), Math.toRadians(robotStatus.getHeading() + e.getBearing())));
		villain.setBearing(e.getBearing());
		villain.setDistance(e.getDistance());
		villain.setEnergy(e.getEnergy());
		villain.setHeading(e.getHeading());
		villain.setVelocity(e.getVelocity());
		villain.setName(e.getName());
		villain.setShootCountHeadOn(enemies.get(e.getName()).getShootCountHeadOn());
		villain.setShootCountLinear(enemies.get(e.getName()).getShootCountLinear());
		villain.setHitCountHeadOn(enemies.get(e.getName()).getHitCountHeadOn());
		villain.setHitCountLinear(enemies.get(e.getName()).getHitCountLinear());
		villain.setEfficiencyHeadOn(villain.getHitCountHeadOn(), villain.getShootCountHeadOn());
		villain.setEfficiencyLinear(villain.getHitCountLinear(), villain.getShootCountLinear());
		villain.setLive(true);
		if (roundNumber == 1) {
			//for the first quarter, shoot in mode 1
			villain.setShootMode(-1);
		} else {
			//for the rest of the match, shoot in whichever mode was more efficienct
			if (villain.getEfficiencyHeadOn() > villain.getEfficiencyLinear()) {
				villain.setShootMode(-1);
			}
			else if (villain.getEfficiencyHeadOn() == villain.getEfficiencyLinear()) {
				villain.setShootMode(1);
			} else {
				villain.setShootMode(1);
			}
		}

		if (!e.getName().equals(enemy.getName())) {
			opponentEnergy = e.getEnergy();
			//if the enemy is not the same as the one before, then update enemy to the new target
		}
		// normal enemy selection: the one closer to you is the most dangerous so attack him
		if(!enemy.getLive() || e.getDistance() < enemy.getDistance()) {
			enemy = villain;
		}

		combat();

		if (getOthers() == 1) {
			//if it is a 1v1 therefore in need of dodging bullets
			if (e.getDistance() > 142) {
				//if the enemy is closer than 143 pixels then dont bother dodging bullets because you will not have time to dodge them
				dodgeBullets();
			}
		}
	}

	//---------------------------------- onStatus event ----------------------------------
	//----------------------------------------------------------------------------------
	public void onStatus(StatusEvent e) {
		this.robotStatus = e.getStatus();
	}

	//---------------------------------- onRobotDeath event ----------------------------------
	//----------------------------------------------------------------------------------------
	public void onRobotDeath(RobotDeathEvent e) {
		if (enemies.containsKey(e.getName())) {
			((Enemy)enemies.get(e.getName())).setLive(false);
		}
	}

	//---------------------------------- onBulletHit event ----------------------------------
	//---------------------------------------------------------------------------------------
	public void onBulletHit(BulletHitEvent e) {
		if (enemy.getName() == e.getName()) {
			if (getRoundNum() == 0) {
				enemies.get(e.getName()).incrementHitCountHeadOn();
			} else {
				if (enemy.getShootMode() == 1) {
					enemies.get(e.getName()).incrementHitCountLinear();
				} else {
					enemies.get(e.getName()).incrementHitCountHeadOn();
				}
			}
		}
	}
	//---------------------------------- combat decides the bullet power and which shoot mode we want to use ----------------------------------
	// ----------------------------------------------------------------------------------------------------------------------------------------
	public void combat() {
		//decide the bullet power depending on the enemy's health. We do not want to waste energy shooting at max firePower at an enemy at 1 energy just because he's close to us
		if (enemy.getEnergy() < 4) {
			firePower = enemy.getEnergy() / 4;
		} else if (enemy.getEnergy() < 16) {
			firePower = (enemy.getEnergy() + 2)/6;
		} else {
			firePower = Math.min(400 / enemy.getDistance(), 3);
		}

		speedOfBullet = 20 - 3 * firePower;

		if (enemy.getShootMode() == 1) {
			//linear prediction of where enemy will be
			double impactTime = getImpactTime(10, 20, 0.01);
			//time needs to be the distance between you and enemy divided by speed of bullet
			Point2D.Double futureXY = getEstimatedPosition(impactTime);

			if (futureXY.x > getBattleFieldWidth()) {
				futureXY.x = getBattleFieldWidth();
			} else if (futureXY.x < 0) {
				futureXY.x = 0.0;
			}

			if (futureXY.y > getBattleFieldHeight()) {
				futureXY.y = getBattleFieldHeight();
			} else if (futureXY.y < 0) {
				futureXY.y = 0.0;
			}
			double absPredictGunTurnAmt = absoluteBearing(robotStatus.getX(), robotStatus.getY(), futureXY.x, futureXY.y);
			turnGunRight(normaliseBearing(absPredictGunTurnAmt - robotStatus.getGunHeading()));
		} else {
			//fire straight at enemy position when scanned
			double absGunTurnAmt = absoluteBearing(robotStatus.getX(), robotStatus.getY(), enemy.getPos().x, enemy.getPos().y);
			turnGunRight(normaliseBearing(absGunTurnAmt - robotStatus.getGunHeading()));
		}

		if (myEnergy > 1 && enemy.getName() != null && getGunHeat() == 0) {
			//only shoot if robot will not disable itself by doing so, if there is an enemy to target and if the gun heating is equal to 0
			fire(firePower);
			if (enemy.getShootMode() == 1) {
				enemy.incrementShootCountLinear();
			} else {
				enemy.incrementShootCountHeadOn();
			}
		}
	}

	//---------------------------------- calculate the time when the bullet will collide with enemy ----------------------------------
	//--------------------------------------------------------------------------------------------------------------------------------
	public double getImpactTime(double bulletTick, double enemyTick, double accuracy) {
		double bTick = bulletTick;
		double eTick = enemyTick;
		int i = 0;

		double distBTick = calcDistance(bTick);

		while ((Math.abs(eTick - bTick)) > accuracy && (i < 15)) {
			i++;
			double distETick = calcDistance(eTick);

			if ((distETick - distBTick) == 0.0) {
				break;
			}
			else {
				double nextTick = eTick - distETick * (eTick - bTick) / (distETick - distBTick);
				bTick = eTick;
				eTick = nextTick;
				distBTick = distETick;
			}
		}
		return eTick;
	}

	//---------------------------------- calculate distance between current position and enemy's future position ----------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------------------
	public double calcDistance(double time) {
		Point2D.Double futureXY = getEstimatedPosition(time);
		double dX = (futureXY.x- robotStatus.getX());
		double dY = (futureXY.y - robotStatus.getY());
		double distance = Math.sqrt(dX*dX + dY*dY) - speedOfBullet * time;
		return distance;
	}

	//---------------------------------- obtain prediction of the enemy's coordinates ----------------------------------
	//------------------------------------------------------------------------------------------------------------------
	public Point2D.Double getEstimatedPosition(double time) {
		double futureX = enemy.getPos().x + enemy.getVelocity() * time * Math.sin(Math.toRadians(enemy.getHeading()));
		double futureY = enemy.getPos().y + enemy.getVelocity() * time * Math.cos(Math.toRadians(enemy.getHeading()));
		Point2D.Double futureXY = new Point2D.Double(futureX, futureY);
		return futureXY;
	}

	//---------------------------------- move using minimum risk movement ----------------------------------
	//------------------------------------------------------------------------------------------------------
	public void doMove() {
		distanceToEnemy = myPos.distance(enemy.getPos());
		double distanceToNextDestination = myPos.distance(nextDestination);
		double addLast = 1 - Math.rint(Math.pow(Math.random(), getOthers()));

		Rectangle2D.Double battleField = new Rectangle2D.Double(30, 30, getBattleFieldWidth() - 60, getBattleFieldHeight() - 60);
		Point2D.Double testPoint;
		int i = 0;

		do {
			testPoint = randomPoint(myPos, Math.min(distanceToEnemy * 0.8, 100 + 200 * Math.random()), 2 * Math.PI * Math.random());
			// generate a random test point in radius of 200 pixels around the robot's current position. Put a cap on
			if(battleField.contains(testPoint) && evaluate(testPoint, addLast) < evaluate(nextDestination, addLast)) {
				nextDestination = testPoint;
			}
		} while(i++ < 200);
		//generate 200 points and keep updating our testPoint to the safest point
		lastPosition = myPos;

		double angle = calcAngle(nextDestination, myPos) - Math.toRadians(robotStatus.getHeading());
		double direction = 1;

		if(Math.cos(angle) < 0) {
			angle += Math.PI;
			direction = -1;
		}

		ahead(distanceToNextDestination * direction);
		turnRight(Math.toDegrees(angle = Utils.normalRelativeAngle(angle)));
		// hitting walls isn't a good idea, but HawkOnFire still does it pretty often

	}

	//---------------------------------- dodge bullets ----------------------------------
	// ----------------------------------------------------------------------------------
	public void dodgeBullets() {
		turnRight(normaliseBearing(enemy.getBearing() + 90));
		//turn the robot to face the closest opponent
		double dropEnergy = opponentEnergy - enemy.getEnergy();
		//if opponent's energy drops by 3 or less, assume it fired
		if (dropEnergy <= 3 && dropEnergy > 0) {
			ahead(30 * movementDirection);
		}
		if (robotStatus.getVelocity() == 0) {
			movementDirection *= -1;
		}
		opponentEnergy = enemy.getEnergy();
		//if robot stops, then change the direction its moving
	}

	//---------------------------------- eval position ----------------------------------
	//-----------------------------------------------------------------------------------
	public static double evaluate(Point2D.Double p, double addLast) {
		double eval = addLast*0.08/p.distanceSq(lastPosition);
		// this is basically here that the bot uses more space on the battlefield. In melee it is dangerous to stay somewhere too long.
		Enumeration<Enemy> e = enemies.elements();

		while (e.hasMoreElements()) {
			Enemy target = (Enemy)e.nextElement();
			double indicator = target.getEnergy() / myEnergy;
			//a simple ratio of enemy energy over our energy can give us an indication of how dangerous an enemy is to us in the current state
			double directionIndicator = (1 + Math.abs(Math.cos(calcAngle(myPos, p) - calcAngle(target.getPos(), p))));
			//Math.abs(Math.cos(calcAngle(myPos, p) - calcAngle(target.pos, p))) is bigger if the moving direction isn't good in relation to a certain bot.
			double force = p.distanceSq(target.getPos());
			//anti gravity force theory applied to enemy robot
			if(target.getLive()) {
				eval += Math.min(indicator, 2) * directionIndicator / force;
			}
		}
		return eval;
	}

	//---------------------------------- random point generator ----------------------------------
	//--------------------------------------------------------------------------------------------
	private static Point2D.Double randomPoint(Point2D.Double testPoint, double distance, double angle) {
		Point2D.Double result = new Point2D.Double(testPoint.x + distance * Math.sin(angle), testPoint.y + distance * Math.cos(angle));
		return result;
	}

	//---------------------------------- calculate angle beteen 2 coordinates ----------------------------------
	//----------------------------------------------------------------------------------------------------------
	private static double calcAngle(Point2D.Double point2,Point2D.Double point1){
		double angle = Math.atan2(point2.x - point1.x, point2.y - point1.y);
		return angle;
	}

	//---------------------------------- absolute bearings ----------------------------------
	//---------------------------------------------------------------------------------------
	public double absoluteBearing(double myX, double myY, double enemyX, double enemyY) {
		double bearing = 0;
		double x = enemyX - myX;
		double y = enemyY - myY;
		double hypotenuse = Point2D.distance(myX, myY, enemyX, enemyY);
		double arcSin = Math.toDegrees(Math.asin(x / hypotenuse));

		if (x > 0 && y > 0) { // both pos: lower-Left
			bearing = arcSin;
		} else if (x < 0 && y > 0) { // x neg, y pos: lower-right
			bearing = 360 + arcSin; // arcsin is negative here, actuall 360 - ang
		} else if (x > 0 && y < 0) { // x pos, y neg: upper-left
			bearing = 180 - arcSin;
		} else if (x < 0 && y < 0) { // both neg: upper-right
			bearing = 180 - arcSin; // arcsin is negative here, actually 180 + ang
		}

		return bearing;
	}

	//---------------------------------- normalising bearings ----------------------------------
	//------------------------------------------------------------------------------------------
	public double normaliseBearing(double angle) {
		while (angle > 180) {
			angle -= 360;
		}
		while (angle < -180) {
			angle += 360;
		}
		return angle;
	}

	//---------------------------------- Enemy ----------------------------------
	//---------------------------------------------------------------------------
	public class Enemy {
		Point2D.Double pos;
		double bearing;
		double distance;
		double heading;
		double velocity;
		double lateralVelocity;
		double angularVelocity;
		double energy;
		String name;
		int shootCountHeadOn = 1;
		int shootCountLinear = 1;
		int hitCountHeadOn = 1;
		int hitCountLinear = 1;
		double efficiencyHeadOn;
		double efficiencyLinear;
		int shootMode = 1;
		boolean live;

		public Point2D.Double getPos() {
			return pos;
		}
		public void setPos(Point2D.Double pos) {
			this.pos = pos;
		}
		public double getBearing() {
			return bearing;
		}
		public void setBearing(double bearing) {
			this.bearing = bearing;
		}
		public double getDistance() {
			return distance;
		}
		public void setDistance(double distance) {
			this.distance = distance;
		}
		public double getHeading() {
			return heading;
		}
		public void setHeading(double heading) {
			this.heading = heading;
		}
		public double getVelocity() {
			return velocity;
		}
		public void setVelocity(double velocity) {
			this.velocity = velocity;
		}
		public double getLateralVelocity() {
			return lateralVelocity;
		}
		public void setLateralVelocity() {
			lateralVelocity = velocity * Math.sin(heading - bearing + robotStatus.getHeading());
		}
		public double getAngularVelocity() {
			return angularVelocity;
		}
		public void setAngularVelocity() {
			angularVelocity = lateralVelocity / distance;
		}
		public double getEnergy() {
			return energy;
		}
		public void setEnergy(double energy) {
			this.energy = energy;
		}
		public String getName() {
			return name;
		}
		public void setName(String name) {
			this.name = name;
		}
		public int getShootCountHeadOn() {
			return shootCountHeadOn;
		}
		public void setShootCountHeadOn(int shootCountHeadOn) {
			this.shootCountHeadOn = shootCountHeadOn;
		}
		public int getShootCountLinear() {
			return shootCountLinear;
		}
		public void setShootCountLinear(int shootCountLinear) {
			this.shootCountLinear = shootCountLinear;
		}
		public int getHitCountHeadOn() {
			return hitCountHeadOn;
		}
		public void setHitCountHeadOn(int hitCountHeadOn) {
			this.hitCountHeadOn = hitCountHeadOn;
		}
		public int getHitCountLinear() {
			return hitCountLinear;
		}
		public void setHitCountLinear(int hitCountLinear) {
			this.hitCountLinear = hitCountLinear;
		}
		public double getEfficiencyHeadOn() {
			return efficiencyHeadOn;
		}
		public void setEfficiencyHeadOn(int hitCountHeadOn, int shootCountHeadOn) {
			efficiencyHeadOn = (double) hitCountHeadOn / shootCountHeadOn;
		}
		public double getEfficiencyLinear() {
			return efficiencyLinear;
		}
		public void setEfficiencyLinear(int hitCountLinear, int shootCountLinear) {
			efficiencyLinear = (double) hitCountLinear / shootCountLinear;
		}
		public int getShootMode() {
			return shootMode;
		}
		public void setShootMode(int shootMode) {
			this.shootMode = shootMode;
		}
		public boolean getLive() {
			return live;
		}
		public void setLive(boolean live) {
			this.live = live;
		}
		public void incrementShootCountHeadOn() {
			shootCountHeadOn++;
		}
		public void incrementShootCountLinear() {
			shootCountLinear++;
		}
		public void incrementHitCountHeadOn() {
			hitCountHeadOn++;
		}
		public void incrementHitCountLinear() {
			hitCountLinear++;
		}
	}
}
