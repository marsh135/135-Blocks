package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.leds.LEDConstants;
import frc.robot.utils.leds.LEDConstants.ImageStates;
import frc.robot.utils.leds.LEDConstants.LEDStates;
import frc.robot.utils.maths.TimeUtil;

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
	public static LEDStates currentLEDState = LEDStates.SINE_WAVE;
	private int[] color = LEDConstants.blueRGB;
	private int flashRateMs = 5000;
	private double currentTimeMs = TimeUtil.getLogTimeSeconds() * 1000.0;
	private int[] altColor = LEDConstants.redRGB;
	//Gif variables
	private static ImageStates currentImageState = ImageStates.debug;
	private static int currentImageIndex = 0;
	private static double lastUpdateTimeMs = TimeUtil.getLogTimeSeconds()
			* 1000.0;

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

	@Override
	public void periodic() {
		// called every 20ms
		currentTimeMs = TimeUtil.getLogTimeSeconds() * 1000.0;
		switch (currentLEDState) {
		case OFF:
			ledBuffer.forEach((i, r, g, b) -> ledBuffer.setRGB(i, 0, 0, 0));
			break;
		case SOLID_COLOR:
			setSolidColor();
			break;
		case RAINBOW:
			setRainbow();
			break;
		case SINE_WAVE:
			setWave();
			break;
		case WAVE2:
			setWave2();
			break;
		case BREATHING:
			setBreathing();
			break;
		case GIF:
			setGif();
			break;
		}
		leds.setData(ledBuffer);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	/**
	 * Turns off the LEDs
	 */
	public void setStateOff() { currentLEDState = LEDStates.OFF; }

	/**
	 * Sets the LEDs to a solid color INTERNAL ONLY
	 */
	private void setSolidColor() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, color[0], color[1], color[2]);
		}
	}

	/**
	 * Sets the LEDs to a rainbow INTERNAL ONLY
	 */
	private void setRainbow() {
		for (int i = 0; i < LEDs.ledBuffer.getLength(); i++) {
			// Calculate the hue based on the current time, flash rate, and position of the LED
			final var hue = (int) ((currentTimeMs / (double) flashRateMs * 180
					+ i * 180 / LEDConstants.ledBufferLength) % 180);
			// Set the value
			LEDs.ledBuffer.setHSV(i, hue, 255, 128);
		}
	}

	/**
	 * Sets the LEDs to a sine wave of 1 color INTERNAL ONLY
	 */
	private void setWave() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			double x = (currentTimeMs / (flashRateMs / 1000) + i)
					* LEDConstants.xDiffPerLed;
			double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0)
					/ 2.0;
			if (Double.isNaN(ratio)) {
				ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent)
						+ 1.0) / 2.0;
			}
			if (Double.isNaN(ratio)) {
				ratio = 0.5;
			}
			int red = (int) (color[0] * ratio);
			int green = (int) (color[1] * ratio);
			int blue = (int) (color[2] * ratio);
			ledBuffer.setRGB(i, red, green, blue);
		}
	}

	/**
	 * Sets the LEDs to a sine wave of 2 colors INTERNAL ONLY
	 */
	private void setWave2() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			double x = (currentTimeMs / (flashRateMs / 1000) + i)
					* LEDConstants.xDiffPerLed;
			double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0)
					/ 2.0;
			if (Double.isNaN(ratio)) {
				ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent)
						+ 1.0) / 2.0;
			}
			if (Double.isNaN(ratio)) {
				ratio = 0.5;
			}
			int red = (int) ((color[0] * (1 - ratio)) + (altColor[0] * ratio));
			int green = (int) ((color[1] * (1 - ratio)) + (altColor[1] * ratio));
			int blue = (int) ((color[2] * (1 - ratio)) + (altColor[2] * ratio));
			ledBuffer.setRGB(i, red, green, blue);
		}
	}

	/**
	 * Sets the LEDs to a breathing effect INTERNAL ONLY
	 */
	private void setBreathing() {
		// Calculate the breathing intensity based on the current time and flash rate
		double intensity = (Math
				.sin(currentTimeMs / (double) flashRateMs * 2 * Math.PI) + 1) / 2;
		// Set each LED to the current color with the calculated intensity
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			int red = (int) (color[0] * intensity);
			int green = (int) (color[1] * intensity);
			int blue = (int) (color[2] * intensity);
			ledBuffer.setRGB(i, red, green, blue);
		}
	}

	/**
	 * Sets the LEDs to a GIF INTERNAL ONLY
	 */
	private void setGif() {
		if (!gifFound(currentImageState)) {
			System.err.println(
					"No images found for ID " + currentImageState.ordinal());
			updateState(LEDStates.OFF);
			return;
		}
		if (currentTimeMs - lastUpdateTimeMs >= flashRateMs) {
			byte[][] currentLedStates = LEDConstants.imageLedStates
					.get(currentImageState.ordinal()).get(currentImageIndex);
			for (var i = 0; i < ledBuffer.getLength(); i++) {
				ledBuffer.setRGB(i, Byte.toUnsignedInt(currentLedStates[i][0]),
						Byte.toUnsignedInt(currentLedStates[i][1]),
						Byte.toUnsignedInt(currentLedStates[i][2]));
			}
			currentImageIndex = (currentImageIndex + 1)
					% LEDConstants.imageLedStates.get(currentImageState.ordinal())
							.size();
			lastUpdateTimeMs = currentTimeMs;
		}
	}

	/**
	 * Resets all time specific variables
	 */
	private void resetTimeSpecificVariables() { lastUpdateTimeMs = 0; }

	/**
	 * Updates all functions to the new flash rate "FlashRate" is defined as the
	 * either the ENTIRE time for a single cycle of the LED state, or, for a gif,
	 * the time between each frame
	 * 
	 * @param flashRateMilliSeconds
	 */
	public void updateFlashRate(int flashRateMilliSeconds) {
		this.flashRateMs = flashRateMilliSeconds;
		resetTimeSpecificVariables();
	}

	/**
	 * Updates all functions to a new color
	 * 
	 * @param color
	 */
	public void updateColor(int[] color) { this.color = color; }

	/**
	 * Updates all functions to a new alternate color
	 * 
	 * @param altColor (Used in wave2)
	 */
	public void updateAltColor(int[] altColor) { this.altColor = altColor; }

	/**
	 * Updates all functions to a new state
	 * 
	 * @param state
	 */
	public void updateState(LEDStates state) {
		if (state == LEDStates.GIF) {
			currentImageIndex = 0;
		}
		currentLEDState = state;
	}

	/**
	 * Updates all functions (really just Gif) to a new image state
	 * 
	 * @param imageState
	 */
	public void updateImageState(ImageStates imageState) {
		currentImageState = imageState;
	}

	/**
	 * Updates all functions to a new state, with additional arguments for the
	 * flash rate, color, and image state.
	 * 
	 * @param state The new LED state.
	 * @param args  Additional arguments that can be:
	 *                 <ul>
	 *                 <li>An int array representing the color. The array should
	 *                 contain three elements representing the RGB values. Each
	 *                 value should be between 0 and 255. Check
	 *                 {@link LEDConstants} or
	 *                 {@link edu.wpi.first.wpilibj.util.Color} for default
	 *                 colors (For WPILib.color, mulitiply by 255 for RGB
	 *                 vals)</li>
	 *                 <li>A double representing the flash rate in seconds.</li>
	 *                 <li>An ImageState object representing the image
	 *                 state.</li>
	 *                 </ul>
	 *                 The order of these arguments does not matter.
	 */
	public void updateLEDState(LEDStates state, Object... args) {
		updateState(state);
		boolean hasColor = false;
		for (Object arg : args) {
			if (arg instanceof int[]) {
				if (!hasColor) {
					updateColor((int[]) arg);
					hasColor = true;
				} else {
					updateAltColor((int[]) arg);
				}
			} else if (arg instanceof Double || arg instanceof Integer) {
				updateFlashRate((int) arg);
			} else if (arg instanceof ImageStates) {
				updateImageState((ImageStates) arg);
			} else {
				throw new IllegalArgumentException("Unexpected argument type: "
						+ arg.getClass().getSimpleName());
			}
		}
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

	public boolean gifFound(ImageStates imageState) {
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

	/**
	 * Does NOT self check. User must look at the images.
	 */
	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> updateLEDState(currentLEDState)).withTimeout(5),
						runOnce(() -> {
							System.out.println("OVER");
						}))
				.until(() -> !getFaults().isEmpty());
	}

	@Override
	public double getCurrent() { return 0; }

	@Override
	public HashMap<String, Double> getTemps() {
		return new HashMap<String, Double>(Map.of("NULL", 0.0));
	}

	@Override
	public void setCurrentLimit(int amps) { return; }
}
