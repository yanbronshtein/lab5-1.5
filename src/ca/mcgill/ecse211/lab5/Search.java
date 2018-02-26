package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Search {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3UltrasonicSensor usSensor;
	private Odometer odometer;
	private Navigation navigation;
	private EV3ColorSensor cSensor;
	private static SampleProvider sampleProvider;
	private static SampleProvider sampleProvider_gyro;
	private EV3GyroSensor gyroSensor;
	private static int sampleSize;
	private static int samplesize_gyro;
	private float[] theta_gyro;

	private static double redMeanR = 14.3539221;
	private static double redMeanG = 2.1460785;
	private static double redMeanB = 1.6264707;
	private static double redsum=redMeanR+redMeanG+redMeanB;
	private static double redMeanR_Ratio = redMeanR/redsum;
	private static double redMeanG_Ratio = redMeanG/redsum;
	private static double redMeanB_Ratio = redMeanB/redsum;
	private static double blueMeanR = 2.3460785;
	private static double blueMeanG = 5.8215687;
	private static double blueMeanB = 7.4019612;
	private static double bluesum=blueMeanR+blueMeanG+blueMeanB;
	private static double blueMeanR_Ratio = blueMeanR/bluesum;
	private static double blueMeanG_Ratio = blueMeanG/bluesum;
	private static double blueMeanB_Ratio = blueMeanB/bluesum;
	private static double yellowMeanR = 23.4617649;
	private static double yellowMeanG = 15.4313731;
	private static double yellowMeanB = 2.4627451;
	private static double yellowsum=yellowMeanR+yellowMeanG+yellowMeanB;
	private static double yellowMeanR_Ratio =yellowMeanR/yellowsum;
	private static double yellowMeanG_Ratio = yellowMeanG/yellowsum;
	private static double yellowMeanB_Ratio = yellowMeanB/yellowsum;
	private static double whiteMeanR = 21.9950981;
	private static double whiteMeanG = 21.6843139;
	private static double whiteMeanB = 15.5676477;
	private static double whitesum=whiteMeanR+whiteMeanG+whiteMeanB;
	private static double whiteMeanR_Ratio = whiteMeanR/whitesum;
	private static double whiteMeanG_Ratio = whiteMeanG/whitesum;
	private static double whiteMeanB_Ratio = whiteMeanB/whitesum;
	private static double tolerance=0.2;
	private static final int FILTER_OUT = 80;
	private int filterControl;

	public Search(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, 
			EV3UltrasonicSensor usSensor, EV3GyroSensor gyroSensor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.navigation = new Navigation(leftMotor, rightMotor);
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
		this.gyroSensor=gyroSensor;
	}
	
	public void search(double LLx, double LLy, double URx, double URy, int TB) {	
		//first point
		double angle=0;
		boolean notFinished = true;
		double distX=0;
		double distY=0;
		navigation.turnTo(0);
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();
		while(notFinished) {
			leftMotor.setSpeed(40);
			rightMotor.setSpeed(40);
			leftMotor.setAcceleration(500);
			rightMotor.setAcceleration(500);
			leftMotor.forward();
			rightMotor.backward();
	
				double dist = getDistanceTurn();
				if(dist< 120) { //4*Tilesize
					leftMotor.stop(true);
					rightMotor.stop();
					Sound.beepSequenceUp();
					angle = odometer.getT();

					findObject(dist, TB);
                    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					while(true) {
						if(Math.abs(LLx - odometer.getX()) < 10 && Math.abs(LLy - odometer.getY()) < 10) {
							leftMotor.stop(true);
							rightMotor.stop();
							break;
						}
					}
					navigation.turnTo(angle+10);
				}
				if(Math.abs(odometer.getT() - 90) <= 8) {
					leftMotor.stop(true);
					rightMotor.stop();
					notFinished = false;
				}
			
		}
		//search at the second corner

	   fetchGyroData();
       navigation.travelTo(URx,LLy);

//        //search at the third corner
        
        navigation.travelTo(URx,URy);
        fetchGyroData();
        navigation.turnTo(270);
        notFinished = true;
        while(notFinished) {
            
            leftMotor.setSpeed(40);
            rightMotor.setSpeed(40);
            leftMotor.setAcceleration(500);
            rightMotor.setAcceleration(500);
            leftMotor.backward();
            rightMotor.forward();
            
                double dist = getDistanceStraight();
                if(dist< 120) {
                    leftMotor.stop(true);
                    rightMotor.stop();
                    Sound.beepSequenceUp();
                    angle = odometer.getT();
                    findObject(dist, TB);
                    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -dist), true);
					while(true) {
						if(Math.abs(URx - odometer.getX()) < 10 && Math.abs(URy - odometer.getY()) < 10) {
							leftMotor.stop(true);
							rightMotor.stop();
							break;
						}
					}
                    navigation.turnTo(angle-10);
                }
           
            if(Math.abs(odometer.getT()-180) <=2) {
                leftMotor.stop(true);
                rightMotor.stop();
                notFinished = false;
            }
        }

	}
	
	public void findObject(double distance, int TB) {
		//leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		//rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		leftMotor.setSpeed(80);
		rightMotor.setSpeed(80);
		leftMotor.forward();
		rightMotor.forward();
		while(true) {
			if(getDistanceStraight() < 4.0) {
				leftMotor.stop(true);
				rightMotor.stop();
				break;
			}
			if(getDistanceStraight() > distance + 12) {
				leftMotor.stop(true);
				rightMotor.stop();
				return;
			}
		}
		int color = getColor();
		if(color == TB) {
			LCD.drawString("Object Detected", 0, 0);
			
			/*Record object position */
			LCD.drawString("x_pos" + odometer.getX() ,0,1);
			LCD.drawString("y_pos" + odometer.getY() ,0,2);
			//LCD.drawString("Red", 0, 1);
			Sound.beep();
		}
		else {
			Sound.twoBeeps();
		}
	}
	public int getColor() {
		sampleProvider = cSensor.getRGBMode();
	    sampleSize = sampleProvider.sampleSize();
	    float []colorSample = new float[sampleSize];
		sampleProvider.fetchSample(colorSample, 0);
		return WhatColor(colorSample[0], colorSample[1], colorSample[2]);

	}
	public  int WhatColor(double R, double G, double B) {
		double sampleSum = R+G+B;
		double sampleR_Ratio=R/sampleSum;
		double sampleG_Ratio=G/sampleSum;
		double sampleB_Ratio=B/sampleSum;
		// checks if the sample is red
		if(whithinRange(sampleR_Ratio,redMeanR_Ratio,tolerance) && whithinRange(sampleG_Ratio,redMeanG_Ratio,tolerance)
				&& whithinRange(sampleB_Ratio,redMeanB_Ratio,tolerance)) {
			// checks if the sample is red
			return 1;
		}else if(whithinRange(sampleR_Ratio,blueMeanR_Ratio,tolerance) && whithinRange(sampleG_Ratio,blueMeanG_Ratio,tolerance)
				&& whithinRange(sampleB_Ratio,blueMeanB_Ratio,tolerance)) {
			// checks if the sample is blue
			return 2;// but set it to yellow
		}else if(whithinRange(sampleR_Ratio,yellowMeanR_Ratio,tolerance) && whithinRange(sampleG_Ratio,yellowMeanG_Ratio,tolerance)
				&& whithinRange(sampleB_Ratio,yellowMeanB_Ratio,tolerance)) {
			// checks if the sample is yellow
			return 3;//but set it to blue
		}else if(whithinRange(sampleR_Ratio,whiteMeanR_Ratio,tolerance) && whithinRange(sampleG_Ratio,whiteMeanG_Ratio,tolerance)
				&& whithinRange(sampleB_Ratio,whiteMeanB_Ratio,tolerance)) {
			//checks if the sample is white
			return 4;
		}else {
			return 0;
		}
	}

	public double getDistanceStraight() {
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance, 0);
		double distance = usDistance[0]*100;

		while (distance >= 200 && filterControl < FILTER_OUT) {
		      // bad value, do not set the distance var, however do increment the
		      // filter value
		      filterControl++;
		 } 
		if (distance >= 200 && filterControl >= FILTER_OUT) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
		    	return distance;
		 } else {
		      // distance went below 255: reset filter and leave
		      // distance alone.
		      filterControl = 0;
		      return distance;
		 }
	}
	public double getDistanceTurn() {
		SampleProvider sampleProvider = usSensor.getMode("Distance"); // usDistance provides samples from
        // this instance
		float[] usDistance = new float[3];
		sampleProvider.fetchSample(usDistance, 0);
	
		List<Double> distance = new ArrayList<Double>();
		sampleProvider.fetchSample(usDistance, 0);
		for(int i = 0; i < 5; i++) {
			sampleProvider.fetchSample(usDistance, 0);
			LCD.drawString("us Distance: " + usDistance[0], 0, 5);
			distance.add((double)usDistance[0]*100);
			Delay.msDelay(30);
		}
		Collections.sort(distance);
		return distance.get(5/2);
	}
	public void fetchGyroData() {
		sampleProvider_gyro = gyroSensor.getAngleMode();
		samplesize_gyro=sampleProvider_gyro.sampleSize();
		theta_gyro=new float[samplesize_gyro];
        odometer.setTheta(theta_gyro[0]);
        }
	public  boolean whithinRange(double value,double target, double tolerance) {
		if(Math.abs(value-target)<=tolerance) {
			return true;
		}else {
			return false;
		}
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}	
}
