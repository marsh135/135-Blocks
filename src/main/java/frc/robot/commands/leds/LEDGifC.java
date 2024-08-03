package frc.robot.commands.leds;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.utils.leds.LEDConstants;
import frc.robot.utils.leds.LEDConstants.ImageStates;

public class LEDGifC extends Command {
	// Declaring Variables
	private LEDs ledS;
	private boolean isFinished;
	private final int delayMs;
	private long lastUpdateTime;
	private int currentImageIndex = 0;
	private final ImageStates imageState;
	/**
	 * Sets the LEDs to display images and swaps images at each delay interval.
	 */
	public LEDGifC(LEDs ledSubsystem, int delayMs,
			ImageStates imageState) {
		this.ledS = ledSubsystem;
		this.delayMs = delayMs;
		this.imageState = imageState;
		addRequirements(ledS);
	}

	@Override
	public void initialize() {
		isFinished = false;
		currentImageIndex = 0;
		if (LEDConstants.imageLedStates.get(imageState.ordinal()).isEmpty()) {
			Logger.recordOutput("LEDS/LEDGifC", "No images found for ID " + imageState.ordinal());
			isFinished = true;
		}
		lastUpdateTime = System.currentTimeMillis();
	}

	@Override
	public void execute() {
		if (isFinished)
			return; // For aborting when no images are found
		long currentTime = System.currentTimeMillis();
		if (currentTime - lastUpdateTime >= delayMs) {
			// Update LEDs with the current image
			byte[][] currentLedStates = LEDConstants.imageLedStates.get(imageState.ordinal())
					.get(currentImageIndex);
			//System.err.println(currentLedStates.length);
			for (var i = 0; i < LEDs.ledBuffer.getLength(); i++) {
				LEDs.ledBuffer.setRGB(i, Byte.toUnsignedInt(currentLedStates[i][0]),
						Byte.toUnsignedInt(currentLedStates[i][1]),
						Byte.toUnsignedInt(currentLedStates[i][2]));
			}
			// Swap to the next image
			//currentImageIndex = 1;
			currentImageIndex = (currentImageIndex + 1)
					% LEDConstants.imageLedStates.get(imageState.ordinal()).size();
			// Set data to buffer
			LEDs.leds.setData(LEDs.ledBuffer);
			lastUpdateTime = currentTime;
		}
	}

	@Override
	public void end(boolean interrupted) { isFinished = true; }

	@Override
	public boolean isFinished() { return isFinished; }
}