package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_state_space.CTREFlywheelS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTREFlywheelC extends Command {
	private final CTREFlywheelS flywheelS;
	private double flywheelSpeed;

	public CTREFlywheelC(CTREFlywheelS flywheelS) {
		this.flywheelS = flywheelS;
		addRequirements(flywheelS);
	}

	@Override
	public void initialize() {
		//do not do anything to flywheels, flywheelS handles it.
	}

	@Override
	public void execute() {
		if (RobotContainer.manipController
				.getLeftTriggerAxis() > CTRESpaceConstants.Controls.kDeadband) {
			flywheelSpeed = RobotContainer.manipController.getLeftTriggerAxis()
					* CTRESpaceConstants.Flywheel.maxRPM;
		} else if (CTRESpaceConstants.Controls.setButton.getAsBoolean()) {
			flywheelSpeed = 4000;
		} else {
			flywheelSpeed = 0;
		}
		flywheelS.setRPM(flywheelSpeed);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}