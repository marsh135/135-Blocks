package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.utils.SimShootNote;

public class VariableAngle extends Command {
	private final IntakeS intakeS;
	private final OutakeS outakeS;
	private int desiredRPM;
	private boolean isFinished = false, isAutonomous;
	Timer timer = new Timer();
	Timer delay = new Timer();

	public VariableAngle(IntakeS intakeS, OutakeS outakeS,
			boolean isAutonomous) {
		this.intakeS = intakeS;
		this.outakeS = outakeS;
		this.isAutonomous = isAutonomous;
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
		if (delay.get() >= 0.75
				|| Math.abs(RobotContainer.manipController.getRightY()) > 0.2
				|| RobotContainer.driveController.getLeftTriggerAxis() > 0.1
				|| RobotContainer.manipController.getLeftTriggerAxis() > 0.1
				|| RobotContainer.driveController.getRightTriggerAxis() > 0.1
				|| RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
			isFinished = true;
		}
		if (timer.get() < 0.15 && !isAutonomous) {
			intakeS.setPrimaryIntake(0.2);
		} else if (Robot.isSimulation() && timer.get() >= .25) {
			intakeS.setPrimaryIntake(0);
			if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 4.5) {
				//  outakeS.setFF(.85); //may not be needed.
				desiredRPM = 6000;
			} else if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 2.4) {
				//  outakeS.setFF(.67); //may not be needed.
				desiredRPM = 4750;
			} else {
				//  outakeS.setFF(.46); //may not be needed.
				desiredRPM = 3300;
			}
			outakeS.setIndividualFlywheelSpeeds(desiredRPM, desiredRPM);
		} else if (timer.get() >= 0.25 && intakeS.isAtState()) {
			intakeS.setPrimaryIntake(0);
			if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 4.5) {
				//  outakeS.setFF(.85); //may not be needed.
				desiredRPM = 6000;
			} else if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 2.4) {
				//  outakeS.setFF(.67); //may not be needed.
				desiredRPM = 4750;
			} else {
				//  outakeS.setFF(.46); //may not be needed.
				desiredRPM = 3300;
			}
			outakeS.setIndividualFlywheelSpeeds(desiredRPM, desiredRPM);
		}
		if (RobotContainer.manipController.getLeftBumper()) {
			intakeS.setPrimaryIntake(-0.5);
			delay.start();
		}
		if (Robot.isSimulation()) {
			if (OutakeS.getFlywheelSpeedDifference() < 100
					&& OutakeS.getBottomSpeedError(desiredRPM) < 100) {
				SimShootNote.shoot();
				isFinished = true;
			} else {
			}
		} else {
			if (OutakeS.getFlywheelSpeedDifference() < 100 && timer.get() >= 0.3
					&& (intakeS.intakeWithinBounds() || intakeS.isAtState(.5))
					&& OutakeS.getBottomSpeedError(desiredRPM) < 150
					&& OutakeS.getTopSpeedError() < 150
					&& Math.abs(CameraS.getXError()) < .05
					&& !RobotContainer.manipController.getAButton()) {
				intakeS.setPrimaryIntake(-0.5);
				delay.start();
			}
			//  SmartDashboard.putNumber("Angle Output", output);
			if (delay.get() < 0.2) {
				//stores values of the intake and distance. Updates every time command is called
				SwerveC.angleOutputDegrees = intakeS.getIntakeAngle();
				SwerveC.variableAngleDistance = CameraS
						.getDistanceFromSpeakerUsingRobotPose();
			}
			intakeS.deployIntake(
					intakeS.createState(CameraS.getDesiredShooterAngle()));
		}
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
