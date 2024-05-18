package frc.robot.commands.leds;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class LEDGifC extends Command {
    // Declaring Variables
    private List<int[][]> imageLedStates;
    private LEDs ledS;
    private boolean isFinished;
    private final int delayMs;
    private long lastUpdateTime;
    private int currentImageIndex = 0;
    private final int ledRows;
	 private final int ledCols;
    /**
     * Sets the LEDs to display images and swaps images at each delay interval.
     */
    public LEDGifC(LEDs ledSubsystem, List<String> imagePaths, int delayMs, int ledRows, int ledCols) {
        this.ledS = ledSubsystem;
        this.delayMs = delayMs;
		  this.ledRows = ledRows;
		  this.ledCols = ledCols;
        addRequirements(ledS);

        // Load and preprocess the images
        imageLedStates = preprocessImages(imagePaths);
    }

    private List<int[][]> preprocessImages(List<String> imagePaths) {
        List<int[][]> imageList = new ArrayList<>();
        for (String path : imagePaths) {
            try {
                BufferedImage image = ImageIO.read(new File(path));
                imageList.add(processImageToLedStates(image));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return imageList;
    }

    private int[][] processImageToLedStates(BufferedImage image) {
		int ledCount = ledRows * ledCols;

		// Resize image to fit the number of LEDs
		BufferedImage resizedImage = new BufferedImage(ledCols, ledRows, BufferedImage.TYPE_INT_RGB);
		resizedImage.getGraphics().drawImage(image, 0, 0, null);

        // Initialize ledStates from the image pixels
        int[][] ledStates = new int[ledCount][3];
        for (int y = 0; y < ledRows; y++) {
            for (int x = 0; x < ledCols; x++) {
                int pixel = resizedImage.getRGB(x, y);
					 int red = (pixel >> 16) & 0xFF;
                int green = (pixel >> 8) & 0xFF;
                int blue = pixel & 0xFF;
                ledStates[x + y * ledCols][0] = red;
					 ledStates[x + y * ledCols][1] = green;
                ledStates[x + y * ledCols][2] = blue;
            }
        }
        return ledStates;
    }

    @Override
    public void initialize() {
        isFinished = false;
        currentImageIndex = 0;
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime >= delayMs) {
            // Update LEDs with the current image
            int[][] currentLedStates = imageLedStates.get(currentImageIndex);
            for (var i = 0; i < LEDs.ledBuffer.getLength(); i++) {
					 LEDs.ledBuffer.setRGB(i, currentLedStates[i][0], currentLedStates[i][1], currentLedStates[i][2]);
            }

            // Swap to the next image
            currentImageIndex = (currentImageIndex + 1) % imageLedStates.size();

            // Set data to buffer
            LEDs.leds.setData(LEDs.ledBuffer);

            lastUpdateTime = currentTime;
        }
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}