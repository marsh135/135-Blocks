package frc.robot.utils.leds;

import java.util.List;

public class LEDConstants {
	//Using doubles to prevent integer division
	public static double
	//The physical port where the LED strip is plugged in 
	ledPort = 9,
			//The number of LEDs in the PANELS (if there are multiple, they are daisy chained and MUST be the same dimensions)
			ledCols = 16, ledRows = 32,
			// amount of LEDs in the light strip
			ledBufferLength = 512,
			//at what LED index the 2nd panel starts (if there is only 1 panel, this is the same as ledBufferLength)		
			ledBufferCutoff = ledBufferLength;
	public static int[] noteRGB = new int[] { 255, 55, 10
	}, redRGB = new int[] { 255, 0, 0
	}, blueRGB = new int[] { 0, 0, 255
	}, greenRGB = new int[] { 0, 255, 0
	}, pinkRGB = new int[] { 255, 192, 203
	}, goldRGB = new int[] { 255, 215, 0
	}, disabledRGB = new int[] { 0, 0, 0
	};

	public enum LEDStates {
		OFF, SOLID_COLOR, RAINBOW, SINE_WAVE, WAVE2, BREATHING, GIF, FIRE
	}

	//Wave constnats
	public static double waveExponent = .5;

	public enum ImageStates {
		debug, gif1, gif2, logo
	}

	public static List<String> imageList = List.of(ImageStates.debug.name(),
			ImageStates.gif1.name(), ImageStates.gif2.name(),
			ImageStates.logo.name());
	//MUST MATCH ORDER IN IMAGESTATES ENUM
	public static List<List<byte[][]>> imageLedStates; //preprocessed on boot
}
