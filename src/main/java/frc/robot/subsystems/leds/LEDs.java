package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Robot;
import frc.robot.utils.leds.LEDConstants;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import javax.imageio.ImageIO;

public class LEDs extends SubsystemBase {
	public static AddressableLED leds;
	public static AddressableLEDBuffer ledBuffer;
	public static AddressableLEDSim ledSim;

	public LEDs() {
		//creates LED objects (the actual LEDs, and a buffer that stores data to be sent to them)
		leds = new AddressableLED(LEDConstants.ledPort);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
		//sets length of the LED strip to buffer length
		leds.setLength(ledBuffer.getLength());
		//starts LED strips
		leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS
		//if the robot is a simulation, create a simulation for the addressableLEDs
		if (Robot.isSimulation()) {
			ledSim = new AddressableLEDSim(leds);
		}
	}

	public static List<byte[][][]> preprocessImages(List<String> imagePaths) {
		List<byte[][][]> imageList = new ArrayList<>();
		for (String path : imagePaths) {
			try {
				BufferedImage image = ImageIO.read(new File(path));
				imageList.add(processImageToLedStates(image));
			}
			catch (IOException e) {
				e.printStackTrace();
			}
		}
		return imageList;
	}

	private static byte[][][] processImageToLedStates(BufferedImage image) {
	 int ledCount = LEDConstants.ledRows * LEDConstants.ledCols;

	 // Resize image to fit the number of LEDs
	 BufferedImage resizedImage = new BufferedImage(LEDConstants.ledCols, LEDConstants.ledRows, BufferedImage.TYPE_INT_RGB);
	 resizedImage.getGraphics().drawImage(image, 0, 0,LEDConstants.ledCols,LEDConstants.ledRows, null);

		// Initialize ledStates from the image pixels
		byte[][][] ledStates = new byte[LEDConstants.totalGifs][ledCount][3];
		for (int j = 0; j < LEDConstants.totalGifs; j++){
			for (int y = 0; y < LEDConstants.ledRows; y++) {
				for (int x = 0; x < LEDConstants.ledCols; x++) {
					 int pixel = resizedImage.getRGB(x, y);
					 byte red = (byte) ((pixel >> 16) & 0xFF);
					 byte green = (byte) ((pixel >> 8) & 0xFF);
					 byte blue = (byte) (pixel & 0xFF);
					 ledStates[j][x + y * LEDConstants.ledCols][0] = red;
					 ledStates[j][x + y * LEDConstants.ledCols][1] = green;
					 ledStates[j][x + y * LEDConstants.ledCols][2] = blue;
				}
		  }
		}
		
		return ledStates;
  }
}
