package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
		CTRESpaceConstants.Controls.goto4000Button
				.whileTrue(new InstantCommand(() -> {
					flywheelSpeed = 4000;
					flywheelS.setRPM(flywheelSpeed);
				})).onFalse(new InstantCommand(() -> {
					flywheelSpeed =0;
					flywheelS.setRPM(flywheelSpeed);
				}));
	}

	@Override
	public void execute() {
		if (flywheelSpeed != 4000) {
			if (RobotContainer.manipController
					.getLeftTriggerAxis() > CTRESpaceConstants.Controls.kDeadband) {
				flywheelSpeed = RobotContainer.manipController.getLeftTriggerAxis()
						* CTRESpaceConstants.Flywheel.maxRPM;
			} else {
				flywheelSpeed = 0;
			}
			flywheelS.setRPM(flywheelSpeed);
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}