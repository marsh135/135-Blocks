package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.leds.LEDGifC;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.leds.LEDConstants;
import frc.robot.utils.leds.LEDConstants.ImageStates;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.Collections;
import java.util.HashMap;
import java.nio.file.Files;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
public class LEDs extends SubsystemChecker {
	public static AddressableLED leds;
	public static AddressableLEDBuffer ledBuffer;
	public static AddressableLEDSim ledSim;
	public static String gifStorage;

	public LEDs() {
		switch (Constants.currentMode) {
		case REAL:
			gifStorage = "/U/images";
			break;
		default:
			ledSim = new AddressableLEDSim(leds);
			gifStorage = "src\\main\\java\\frc\\robot\\utils\\leds\\images";
			break;
		}
		//creates LED objects (the actual LEDs, and a buffer that stores data to be sent to them)
		leds = new AddressableLED(LEDConstants.ledPort);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
		//sets length of the LED strip to buffer length
		leds.setLength(ledBuffer.getLength());
		//starts LED strips
		leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS -N
		//if the robot is a simulation, create a simulation for the addressableLEDs
		LEDConstants.imageLedStates = preprocessImages(LEDConstants.imageList);
	}

	public static List<List<byte[][]>> preprocessImages(List<String> gifPaths) {
		List<List<byte[][]>> imageList = new ArrayList<>();
		for (int i = 0; i < gifPaths.size(); i++) {
			List<byte[][]> gifImages = new ArrayList<>();
			String gifPath;
			if (Constants.currentMode == Mode.REAL) {
				gifPath = gifStorage + "/" + gifPaths.get(i);
			} else {
				gifPath = gifStorage + "\\" + gifPaths.get(i);
			}
			List<String> filePaths = new ArrayList<>();
			try {
				Files.walkFileTree(Paths.get(gifPath),
						new SimpleFileVisitor<Path>() {
							@Override
							public FileVisitResult visitFile(Path file,
									BasicFileAttributes attrs) throws IOException {
								filePaths.add(file.toString());
								System.err.println(file.toString());
								return FileVisitResult.CONTINUE;
							}
						});
			}
			catch (IOException e) {
				e.printStackTrace();
			}
			for (String path : filePaths) {
				try {
					BufferedImage image = ImageIO.read(new File(path));
					gifImages.add(processImageToLedStates(image));
				}
				catch (IOException e) {
					e.printStackTrace();
				}
			}
			imageList.add(gifImages);
		}
		return imageList;
	}
	public boolean imageFound(ImageStates imageState) {
		return !LEDConstants.imageLedStates.get(imageState.ordinal()).isEmpty();
	}
	private static byte[][] processImageToLedStates(BufferedImage image) {
		int ledCount = LEDConstants.ledRows * LEDConstants.ledCols;
		// Resize image to fit the number of LEDs
		BufferedImage resizedImage = new BufferedImage(LEDConstants.ledCols,
				LEDConstants.ledRows, BufferedImage.TYPE_INT_RGB);
		resizedImage.getGraphics().drawImage(image, 0, 0, LEDConstants.ledCols,
				LEDConstants.ledRows, null);
		// Initialize ledStates from the image pixels
		byte[][] ledStates = new byte[ledCount][3];
		for (int y = 0; y < LEDConstants.ledRows; y++) {
			for (int x = 0; x < LEDConstants.ledCols; x++) {
				int pixel = resizedImage.getRGB(x, y);
				byte red = (byte) ((pixel >> 16) & 0xFF);
				byte green = (byte) ((pixel >> 8) & 0xFF);
				byte blue = (byte) (pixel & 0xFF);
				ledStates[x + y * LEDConstants.ledCols][0] = red;
				ledStates[x + y * LEDConstants.ledCols][1] = green;
				ledStates[x + y * LEDConstants.ledCols][2] = blue;
			}
		}
		return ledStates;
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	/**
	 * Does NOT self check. User must look at the images.
	 */
	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				new LEDGifC(this, 20, ImageStates.debug).withTimeout(5),
				runOnce(() -> {
					System.out.println("OVER");
				})).until(() -> !getFaults().isEmpty());
	}

	@Override
	public double getCurrent() { return 0; }

	@Override
	public HashMap<String, Double> getTemps() { return new HashMap<String, Double>(Map.of("NULL", 0.0));}

	@Override
	public void setCurrentLimit(int amps) { return; }
}
