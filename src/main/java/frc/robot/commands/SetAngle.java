package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;

public class SetAngle extends Command {
	private final IntakeS intakeS;
	private final OutakeS outakeS;
	private boolean isFinished = false;
	private double desAngle;
	Timer timer = new Timer();
	Timer delay = new Timer();

	public SetAngle(IntakeS intakeS, OutakeS outakeS, double desAngle) {
		this.intakeS = intakeS;
		this.outakeS = outakeS;
		this.desAngle = desAngle;
		addRequirements(intakeS, outakeS);
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		delay.reset();
		isFinished = false;
	}

	@Override
	public void execute() {
		if (delay.get() >= 1
				|| Math.abs(RobotContainer.manipController.getRightY()) > 0.2
				|| RobotContainer.driveController.getLeftTriggerAxis() > 0.1
				|| RobotContainer.manipController.getLeftTriggerAxis() > 0.1
				|| RobotContainer.driveController.getRightTriggerAxis() > 0.1
				|| RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
			isFinished = true;
		}
		if (timer.get() < 0.15) {
			intakeS.setPrimaryIntake(0.2);
		} else if (timer.get() >= 0.25 && intakeS.isAtState()) {
			intakeS.setPrimaryIntake(0);
			double outakeSpeed = 6000; //was 0.85 + outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 6000);
			outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
		}
		if (RobotContainer.manipController.getLeftBumper()) {
			intakeS.setPrimaryIntake(-0.5);
			delay.start();
		}
		if (OutakeS.getFlywheelSpeedDifference() < 100 && timer.get() >= 0.3
				&& OutakeS.getBottomSpeedError(6000) < 150
				&& OutakeS.getTopSpeedError() < 150
				&& !RobotContainer.manipController.getAButton()
				&& intakeS.isAtState()) {
			intakeS.setPrimaryIntake(-0.5);
			delay.start();
		}
		//SmartDashboard.putNumber("Angle Output", output);
		intakeS.deployIntake(intakeS.createState(desAngle));
	}

	@Override
	public void end(boolean interrupted) {
		intakeS.setPrimaryIntake(0);
		outakeS.setIndividualFlywheelSpeeds(0, 0);
		timer.stop();
		timer.reset();
		delay.stop();
		delay.reset();
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
