package frc.robot.utils.leds;

import java.util.List;

public class LEDConstants {
	public static int
	//The physical port where the LED strip is plugged in 
	ledPort = 9, ledCols = 16, ledRows = 32,
			// amount of LEDs in the light strip
			ledBufferLength = ledCols * ledRows, totalGifs = 2;
	//HSV Colors with values stored in arrays
	public static int[] noteHSV = new int[] { 12, 255, 100
	}, redHSV = new int[] { 0, 255, 100
	}, blueHSV = new int[] { 120, 255, 100
	}, greenHSV = new int[] { 50, 255, 100
	}, pinkHSV = new int[] { 147, 255, 255
	}, goldHSV = new int[] { 23, 255, 100
	}, disabledHSV = new int[] { 0, 0, 0
	};

	public enum ImageStates {
		debug, gif1, gif2
	}

	public static List<String> imageList = List.of(ImageStates.debug.name(),
			ImageStates.gif1.name(), ImageStates.gif2.name());
	//MUST MATCH ORDER IN IMAGESTATES ENUM
	public static List<List<byte[][]>> imageLedStates; //preprocessed on boot
}
