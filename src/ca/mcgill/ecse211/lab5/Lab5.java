/**
 * Lab5.java
 * 
 */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.UltrasonicLocalizer.LocalizationType;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3GyroSensor;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final EV3UltrasonicSensor  usSensor = new EV3UltrasonicSensor
			(LocalEV3.get().getPort("S1"));
	public static final EV3GyroSensor gyroSensor= new EV3GyroSensor(LocalEV3.get().getPort("S3"));
	public static final double WHEEL_RAD = 2.16;
	public static final double TRACK = 12.2;
	
	public static final double TILE_SIZE = 30.48;
	public static final int ROTATE_SPEED = 100;

	/**Lower corner x */
	public static int LLx = 3;
	/**Lower corner y */
	public static int LLy = 3;
	/**Upper corner x */
	public static int URx = 7;
	
	/**Upper corner y */public static int URy = 7;
	/**Test block red=1, blue = 2, yellow=3, white=4 */
	public static int TB = 4;
	
	/**starting corner */
	public static int SC = 0;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		Navigation navigation = new Navigation();
		UltrasonicLocalizer ultrasoniclocalizer = new UltrasonicLocalizer(LocalizationType.fallingEdge,
				odometer, usSensor);
		Search search = new Search( odometer, usSensor, gyroSensor);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer);
		//OdometryCorrection odometryCorrection = new OdometryCorrection(); 
		Display odometryDisplay = new Display(lcd); // No need to change
		//Avoid avoid = new Avoid(leftMotor, rightMotor);
		do{
			lcd.clear();

			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString(" Iden- | The    ", 0, 1);
			lcd.drawString(" fy    | whole  ", 0, 2);
			lcd.drawString("       | lab ", 0, 3);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			lcd.clear();
			
			
			while(true) {
				int color = search.getColor();
				//red
				if(color == 1) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Red", 0, 1);
				}
				//blue
				if(color == 2) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Blue", 0, 1);
				}
				//yellow
				if(color == 3) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("Yellow", 0, 1);
				}
				//white
				if(color == 4) {
					lcd.drawString("Object Detected", 0, 0);
					lcd.drawString("White", 0, 1);
				}
				if(Button.getButtons() == Button.ID_ESCAPE) {
					System.exit(0);
				}
			}

		} else {
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("Press a button", 0, 0);
				lcd.drawString(" to start        ", 0, 1);
				lcd.drawString("localization  ", 0, 2);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			ultrasoniclocalizer.UltrasonicLocalization();
			lightLocalizer.LLocalization(SC);
			search.fetchGyroData();
			do {
				buttonChoice = Button.waitForAnyPress();
			} while (buttonChoice != Button.ID_LEFT);

			if(SC == 2 || SC == 3) {
				navigation.travelTo(LLx, URy);
				search.fetchGyroData();
				navigation.travelTo(LLx, LLy);
			}else {
				navigation.travelTo(LLx*TILE_SIZE,LLy*TILE_SIZE);
			}
			Sound.beep();
			search.search(LLx*TILE_SIZE, LLy*TILE_SIZE, URx*TILE_SIZE, URy*TILE_SIZE, TB);
			navigation.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);


			
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
