package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	public static final EV3LargeRegulatedMotor leftMotor =
			Lab5.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor =
			Lab5.rightMotor;
	private Odometer odometer;
	private EV3ColorSensor cSensor;
	private static SampleProvider sampleProvider;
	private static int sampleSize;
	private static final int ROTATE_SPEED =100;
	private double x;
	private double y;
	private double currentX;
	private double currentY;
	private double dx;
	private double dy;
	private double distance;
	private double thetaCorrection;
	private double TILE_SIZE = Lab5.TILE_SIZE;
	private double WHEEL_RAD = Lab5.WHEEL_RAD;
	private double distToSensor = -15;
	/**incident angle to first, second, third, and fourth gridlines */
	private double angle[] = new double[4]; 
	private double thetay, thetax;
	
	public LightLocalizer(Odometer odometer) {
		
		this.odometer = odometer;
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	}
	public void LLocalization(int SC) {
		if(SC==0){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(distToSensor* Math.cos(Math.toRadians(thetay)));
			odometer.setY(distToSensor* Math.cos(Math.toRadians(thetax)));

			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
		}
		if(SC==1){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(distToSensor* Math.cos(Math.toRadians(thetay)));
			odometer.setY(distToSensor* Math.cos(Math.toRadians(thetax)));
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(7*TILE_SIZE, TILE_SIZE, 270);
		}
		if(SC==2){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(distToSensor* Math.cos(Math.toRadians(thetay)));
			odometer.setY(distToSensor* Math.cos(Math.toRadians(thetax)));
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
		}
		if(SC==3){
			lightLocalization();
			thetay = (angle[0]-angle[2])/2;
			thetax = (angle[1]-angle[3])/2;
    			odometer.setX(distToSensor* Math.cos(Math.toRadians(thetay)));
			odometer.setY(distToSensor* Math.cos(Math.toRadians(thetax)));
			travelTo(0,0);
			turnTo(0);
			odometer.setXYT(TILE_SIZE, 7*TILE_SIZE, 90);
		}
	}
	
	/**Use RGB values to detect black line. 
	 * */
	public void lightLocalization() {
		int counter = 0;
		cSensor.setFloodlight(lejos.robotics.Color.RED);
	      //get the RGB value of the sensor
	    sampleProvider = cSensor.getRGBMode();
	    sampleSize = sampleProvider.sampleSize();
	    float []colorSample = new float[sampleSize];
	    	turnTo(45);	
	    	leftMotor.rotate(convertDistance(WHEEL_RAD, 5 ), true);     
	    	rightMotor.rotate(convertDistance(WHEEL_RAD, 5 ), false);
	    	leftMotor.backward();
	    	rightMotor.forward();
		while(true) {
			sampleProvider.fetchSample(colorSample, 0);
			
			/*Count number of blacklines*/
			if(BlackLine(colorSample[0], colorSample[1], colorSample[2])) {
				angle[counter] = odometer.getT();
				Sound.beep();
				counter++;
				if(counter == 4) {
					break;
				}
			}
		}
	}
	
	
	/**Method Discards RBG values that could not correspond to black line
	 * @returns true if correct range for all values  */
	public boolean BlackLine(float R,float G, float B) {
		  if(R<=0.002 || G<=0.002 || B<=0.002) {
			  //false if the values are too low, happens when looking at something far
			  return false;
		  }
		  else if(R<=0.19 && G<=0.11 && B<=0.085) {
			  // true if the values are within the range of a blackline
			  return true;
		  }
		  else {
			  // false if doesn't match a blackline
			  return false;
		  }
	}
	public void travelTo(double x, double y) {
		double t;
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
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}
		
	public void turnTo(double theta) {
		double turnAngle;
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
		leftMotor.rotate(convertAngle(WHEEL_RAD, Lab5.TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, Lab5.TRACK, turnAngle), false);
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}
