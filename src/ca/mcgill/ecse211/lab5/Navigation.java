package ca.mcgill.ecse211.lab5;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private Odometer odometer;
	private double t;
	
	private double turnAngle;
	public static final EV3LargeRegulatedMotor leftMotor =
			Lab5.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor =
			Lab5.rightMotor;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;
		
	public Navigation() {
		
		try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}
	/**
	 * Method to calculate distance to point, choice min angle and go straight to it
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		this.x = x;
		this.y = y;
		
		currentX = odometer.getX();
		currentY = odometer.getY();
		dx = x - currentX;
		dy = y - currentY;
		
		distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy, 2));
		t = Math.toDegrees(Math.atan2(dx, dy));
		
		turnTo(t);
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), false);
		
		
	}
	
	public void turnTo(double theta) {
		turnAngle = theta - odometer.getT();
		//calculate the minimal angle
		if(turnAngle < -180) {
			turnAngle = turnAngle + 360;
		}
		else if(turnAngle > 180) {
			turnAngle = turnAngle - 360;
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turnAngle), false);
	}
	
	/**Method checks if either left or right motor moving
	 * @returns true if at least one movign */
	public boolean isNavigating(){
		return leftMotor.isMoving() || rightMotor.isMoving();
	}
	
	
	/**
	 * Determine how much the motor must rotate for vehicle to reach a certain distance
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Determine the angle wheel motors need to rotate to for robot to turn to desired angle 
	 * 
	 * @param radius
	 * @param TRACK
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
