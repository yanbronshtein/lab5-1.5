package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class UltrasonicLocalizer {
	public enum LocalizationType{fallingEdge, risingEdge};
	private EV3LargeRegulatedMotor leftMotor = Lab5.leftMotor;
	private EV3LargeRegulatedMotor rightMotor = Lab5.leftMotor;
	private EV3UltrasonicSensor usSensor;
	private double WHEEL_RAD = Lab5.WHEEL_RAD;
	private double TRACK =Lab5.TRACK;
	private Odometer odometer;
	private LocalizationType type;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED =100;
	private static final int DETECT_DISTANCE = 45;
	private double distance[];
	
	
	public UltrasonicLocalizer(LocalizationType type,
			Odometer odometer, EV3UltrasonicSensor usSensor) {
		
		this.type = type;
		this.odometer = odometer;
		this.usSensor = usSensor;
		
	}
	public void UltrasonicLocalization() {
		double alpha, beta;
		//rising edge
		if(type == LocalizationType.risingEdge) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					break;
				}
			}
			//rotate to the left until it detects a rising edge
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					alpha = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Delay.msDelay(1000);
			sleep(1000);
			//keep rotating to the left until it sees a wall
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					beta = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			if(alpha < beta) {
				odometer.setTheta((45-(alpha+beta)/2) + odometer.getT());
			}
			else {
				odometer.setTheta((225-(alpha+beta)/2) + odometer.getT());
			}
			turnTo(0);			
		}
		//falling edge
		else{
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			//rotate to the left until it detects a falling edge
			while(true) {
				if(getDistance() > DETECT_DISTANCE) {
					break;
				}
			}
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					alpha = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			Delay.msDelay(1000);
			sleep(1000);
			//keep rotating to the left until it sees a wall
			while(true) {
				if(getDistance() < DETECT_DISTANCE) {
					beta = odometer.getT();
					Sound.beep();
					leftMotor.stop(true);
					rightMotor.stop();
					break;
				}
			}
			if(alpha < beta) {
				odometer.setTheta((45-(alpha+beta)/2) + odometer.getT());
			}
			else {
				odometer.setTheta((225-(alpha+beta)/2) + odometer.getT());
			}
			turnTo(0);			
		}
		
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
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle), false);
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
	/**This method is used for when we are locating the blocks. 
	 * It applies a median filter to discard garbage US sensor readings */
	public double getDistance() {
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		List<Double> distance = new ArrayList<Double>();
		//sampleProvider.fetchSample(usDistance, 0);
		for(int i = 0; i < 5; i++) {
			sampleProvider.fetchSample(usDistance, 0);
			LCD.drawString("us Distance: " + usDistance[0], 0, 5);
			//distance[i] = usDistance[0]*100;
			distance.add((double)usDistance[0]*100);
			Delay.msDelay(30);
		}
		Collections.sort(distance);
		return distance.get(5/2);
	}
	public static void sleep(int time) {
		long a = System.currentTimeMillis();
		long b = System.currentTimeMillis();
		while( (b-a) <= time) {
			b = System.currentTimeMillis();
		}
	}
}
