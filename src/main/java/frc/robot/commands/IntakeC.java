package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;

public class IntakeC extends Command {
	private final IntakeS intakeS;

	public IntakeC(IntakeS intakeS) {
		this.intakeS = intakeS;
		addRequirements(intakeS);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double intakeSpeed = 0;
		// Both driver and manip can control intake, driver takes prescendence when intaking, manip takes prescendence when outaking
		if (RobotContainer.manipController.getLeftTriggerAxis() > 0.1) {
			intakeSpeed = RobotContainer.manipController.getLeftTriggerAxis();
		}
		if (RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
			intakeSpeed = -1
					* RobotContainer.manipController.getRightTriggerAxis();
		}
		if (RobotContainer.driveController.getLeftTriggerAxis() > 0.1) {
			intakeSpeed = RobotContainer.driveController.getLeftTriggerAxis();
		}
		if (RobotContainer.driveController.getRightTriggerAxis() > 0.1) {
			intakeSpeed = -1
					* RobotContainer.driveController.getRightTriggerAxis();
		}
		intakeSpeed = Math.pow(intakeSpeed, 2) * (intakeSpeed < 0 ? -1 : 1);
		if (RobotContainer.manipController.getLeftBumper()) {
			intakeSpeed = -0.5;
		}
		double deployIntakeSpeed = RobotContainer.manipController.getRightY();
		if (Math.abs(deployIntakeSpeed) < 0.1)
			deployIntakeSpeed = 0;
		if ((RobotContainer.manipController.getXButton()
				|| RobotContainer.manipController.getRightBumper())
				&& RobotContainer.isDriving().getAsBoolean()) {
			intakeS.deployIntake(intakeS.outsideBotState());
		}
		if (RobotContainer.manipController.getAButton()) {
			intakeS.deployIntake(intakeS.insideBotState());
		}
		intakeS.setPrimaryIntake(intakeSpeed * 1);
		//implement a way to take a percentage output (0 - 1 ) to a velocity for the deploy intake. 
		// Predict .02seconds later to the current position with the given velocity
		// Feed that into the state position
		// use the velocity given for state velocity 
		// Remember, the state uses DEGREES.
		//intakeS.deployIntake(deployIntakeSpeed * 1);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
