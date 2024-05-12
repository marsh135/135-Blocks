package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDStripS extends SubsystemBase {
	double timesRan = 0;
	AddressableLEDBuffer ledBuffer;
	public static AddressableLED leds;
	boolean runSineWave = false;
	public static int initialLoopValue = 0, schedulerCount = 0,
			m_rainbowFirstPixelHue;
	public static double breathingLoopValue = 0;

	public LEDStripS() {
		//creates LED objects  
		leds = new AddressableLED(LEDConstants.ledPort);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
		//sets length of the LED strip to buffer length
		leds.setLength(ledBuffer.getLength());
		//starts LED strips
		leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS
		for (var i = 0; i < LEDConstants.sinePeriod; i++) {
			LEDConstants.ledStates[i] = (int) Math.floor(
				Math.abs(
					Math.sin(
					((
						i * Math.PI / LEDConstants.sinePeriod) + initialLoopValue))
					)
				* 255);
		}
	}

	@Override
	public void periodic() {
		/*NOTE: This code does not have a designated indicator for when the AutoLock program is running*/
		schedulerCount += 1;
		//stops integer overflow (even though it'd take over a year of non-stop operation for that to happen)
		schedulerCount = schedulerCount % (LEDConstants.sineWaveUpdateCycles);
		runSineWave = (schedulerCount == 0);
		//if there is a note stored in the intake, set it to a constant note color
		// if it's disabled make it so LEDs are off unless navx is disconnected. If navx is disconnected run colorwave
		if (DriverStation.isDisabled()) {
			if (!SwerveS.fieldOriented) {
				setConstantColors(LEDConstants.disabledHSV);
			} else {
				setColorWave(LEDConstants.goldHSV, runSineWave);
			}
		} else { //if its enabled
			if (OutakeS.SysIDTestRunning) {
				setLEDSBreathing(LEDConstants.pinkHSV, runSineWave);
			}
			//if it is trying to autolock
			else if (SwerveS.autoLock) {
				//if its locked on, set to constant green
				if (CameraS.aprilTagVisible() && CameraS.robotInRange()) {
					setConstantColors(LEDConstants.greenHSV);
				} else {
					setConstantColors(LEDConstants.redHSV);
				}
			} else if (!SwerveS.fieldOriented) {
				setConstantColors(LEDConstants.blueHSV);
			} else {
				rainbow();
			}
		}
	}

	public void rainbow() {
		if (runSineWave) {
			// For every pixel
			for (var i = 0; i < ledBuffer.getLength(); i++) {
				// shape is a circle so only one value needs to precess
				final var hue = (m_rainbowFirstPixelHue
						+ (i * 180 / LEDConstants.ledBufferLength)) % 180;
				// Set the value
				ledBuffer.setHSV(i, hue, 255, 128);
			}
			// Increase by to make the rainbow "move"
			m_rainbowFirstPixelHue += 3;
			// Check bounds
			m_rainbowFirstPixelHue %= 180;
			leds.setData(ledBuffer);
			return;
		} else {
			return;
		}
	}

	public void setConstantColors(int[] LEDColors) {//Essentially designed to make all the LEDs a constant color 
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			ledBuffer.setHSV(i, LEDColors[0], LEDColors[1], LEDColors[2]);
		}
		leds.setData(ledBuffer);
	}

	public void setColorWave(int[] LEDColors, boolean run) {//value is basically how dark it is, is controlled by the wave function
		if (run) {
			for (var i = 0; i < (ledBuffer.getLength()); i++) {
				final int value = LEDConstants.ledStates[(i + initialLoopValue)
						% LEDConstants.sinePeriod];
				ledBuffer.setHSV(i, LEDColors[0], LEDColors[1], value);
			}
			//offset by one "notch" each time
			initialLoopValue += 1;
			//Prevents any kind of integer overflow error happening (prob would take the robot running for a year straight or something like that to reach, )
			initialLoopValue %= LEDConstants.sinePeriod;
			//sets data to buffer
			leds.setData(ledBuffer);
		}
	}

	public void setLEDSBreathing(int[] LEDColors, boolean run) {
		breathingLoopValue += .25;
		breathingLoopValue %= LEDConstants.sinePeriod;
		final int value = LEDConstants.ledStates[(int) Math.floor(breathingLoopValue)];
		if (run) {
			for (var i = 0; i < (ledBuffer.getLength()); i++) {
				ledBuffer.setHSV(i, LEDColors[0], LEDColors[1], value);
				leds.setData(ledBuffer);
			}
			//offset by one "notch" each time
			//Prevents any kind of integer overflow error happening (prob would take the robot running for a year straight or something like that to reach, )
			//sets data to buffer
			leds.setData(ledBuffer);
		}
	}
}
