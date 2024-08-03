package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class LEDBreathingC extends Command {
	//Declaring Variables
	private final LEDs ledSubsystem;
	private final int[] color;
	private boolean isFinished;
	private double breathingLoopValue;
	private final double breathingPeriod;
	private int[] breathingLEDStates;
	private final double timePerStep = 0.02; // Time per step in seconds
	private final int breathingLoopPeriodSteps; // Number of steps in the breathing cycle

	/**
	 * Changes the brightness of all the LEDs to make them look like they are
	 * breathing (kind of like the Razer effect)
	 * 
	 * @param color                      The color of the LEDs
	 * @param subsystem                  The LEDs subsystem
	 * @param breathingTimeToFullSeconds The time it takes for the LEDs to go
	 *                                      from off to full brightness (and same
	 *                                      for full to off)
	 */
	public LEDBreathingC(int[] color, LEDs subsystem,
			double breathingTimeToFullSeconds) {
		this.runsWhenDisabled();
		ledSubsystem = subsystem;
		addRequirements(ledSubsystem);
		this.color = color;
		this.breathingPeriod = breathingTimeToFullSeconds;
		this.breathingLoopPeriodSteps = (int) Math
				.round(breathingPeriod / timePerStep);
		breathingLEDStates = new int[breathingLoopPeriodSteps];
	}

	@Override
	public void initialize() {
		isFinished = false;
		breathingLoopValue = 0;
		isFinished = false;
		// Adjust the argument to the sine function to complete a full cycle over a period of π
		for (var i = 0; i < breathingLoopPeriodSteps; i++) {
			breathingLEDStates[i] = (int) Math.floor(Math.abs(
					Math.sin((i * 2.0 * Math.PI / breathingLoopPeriodSteps))) * 255);
		}
	}

	@Override
	public void execute() {
		breathingLoopValue += .25;
		breathingLoopValue %= breathingLoopPeriodSteps;
		final int value = breathingLEDStates[(int) Math
				.floor(breathingLoopValue)];
		for (var i = 0; i < (LEDs.ledBuffer.getLength()); i++) {
			LEDs.ledBuffer.setHSV(i, color[0], color[1], value);
			LEDs.leds.setData(LEDs.ledBuffer);
		}
		//offset by one "notch" each time
		//Prevents any kind of integer overflow error happening (prob would take the robot running for a year straight or something like that to reach, )
		//sets data to buffer
		LEDs.leds.setData(LEDs.ledBuffer);
	}

	@Override
	public void end(boolean isFinished) { isFinished = true; }

	@Override
	public boolean isFinished() { return isFinished; }
}
