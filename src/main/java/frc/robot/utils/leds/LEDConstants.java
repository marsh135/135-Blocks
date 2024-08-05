package frc.robot.utils.leds;

import java.util.List;

public class LEDConstants {
	public static int
	//The physical port where the LED strip is plugged in 
	ledPort = 9, ledCols = 16, ledRows = 32,
			// amount of LEDs in the light strip
			ledBufferLength = ledCols * ledRows, totalGifs = 2;
	public static int[] noteRGB = new int[] { 255, 55, 10
	}, redRGB= new int[] { 255, 0, 0
	}, blueRGB = new int[] { 0,0,255
	}, greenRGB = new int[] { 0,255,0
	}, pinkRGB = new int[] { 255, 192, 203
	}, goldRGB = new int[] { 255,215,0
	}, disabledRGB = new int[] { 0, 0, 0
	};
	public enum LEDStates{
		OFF,SOLID_COLOR,RAINBOW,SINE_WAVE,WAVE2,BREATHING,GIF, FIRE
	}
	//Wave constnats
	public static double waveExponent = 0.4,
	 xDiffPerLed = (2.0 * Math.PI) / ledBufferLength;
	public enum ImageStates {
		debug, gif1, gif2
	}

	public static List<String> imageList = List.of(ImageStates.debug.name(),
			ImageStates.gif1.name(), ImageStates.gif2.name());
	//MUST MATCH ORDER IN IMAGESTATES ENUM
	public static List<List<byte[][]>> imageLedStates; //preprocessed on boot
}
