package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.utils.SimShootNote;

public class VariableSpeed extends Command {
	private final IntakeS intakeS;
	private final OutakeS outakeS;
	private boolean isFinished = false, isAutonomous;
	Timer timer = new Timer();
	Timer delay = new Timer();

	public VariableSpeed(IntakeS intakeS, OutakeS outakeS,
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
		intakeS.deployIntake(intakeS.outsideBotState());
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
		} else if (timer.get() >= 0.25) {
			intakeS.setPrimaryIntake(0);
			double outakeSpeed;
			outakeSpeed = 4000; //was 0.49 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.topFlywheelEncoder.getVelocity(), 4000), -0.1, 0.1);
			outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
		}
		if (RobotContainer.manipController.getLeftBumper()) {
			intakeS.setPrimaryIntake(-0.5);
			if (Robot.isSimulation()){
				SimShootNote.shoot();
			}
			delay.start();
		}
		if (CameraS.robotInRange() && OutakeS.getFlywheelSpeedDifference() < 100
				&& timer.get() >= 0.3 && OutakeS.getBottomSpeedError(4000) < 150
				&& OutakeS.getTopSpeedError() < 150
				&& Math.abs(CameraS.getXError()) < 3
				&& !RobotContainer.manipController.getAButton()
				&& intakeS.isAtState()) {
			intakeS.setPrimaryIntake(-0.5);
			if (Robot.isSimulation()){
				SimShootNote.shoot();
			}
			delay.start();
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
