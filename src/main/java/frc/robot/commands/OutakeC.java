package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;
import frc.robot.utils.SimShootNote;

public class OutakeC extends Command {
	private final OutakeS outakeS;
	double outakeSpeed;

	public OutakeC(OutakeS outakeS) {
		this.outakeS = outakeS;
		addRequirements(outakeS);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (RobotContainer.manipController.getStartButton()) {
			if (RobotContainer.manipController.getLeftBumperPressed()) {
				outakeSpeed += 100;
			}
			if (RobotContainer.manipController.getRightBumperPressed()) {
				outakeSpeed -= 100;
			}
		} else {
			if (RobotContainer.manipController.getRightBumper()) {
				double topWheelSpeed = 2414; // 34%
				double bottomWheelSpeed = 2201; //31%
				//double topWheelSpeed = OutakeConstants.idealPercentTop + outakeS.shooterPID.calculate(OutakeS.topFlywheelEncoder.getVelocity(), OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentTop);
				//double bottomWheelSpeed = OutakeConstants.idealPercentBottom + outakeS.shooterPID.calculate(OutakeS.bottomFlywheelEncoder.getVelocity(),OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentBottom);
				outakeS.setIndividualFlywheelSpeeds(topWheelSpeed,
						bottomWheelSpeed);
				if (Robot.isSimulation()) {
					SimShootNote.shoot();
				}
			}
			//for speaker
			else {
				if (RobotContainer.driveController.getLeftBumper() == true) {
					outakeSpeed = -1775; //was -.25
				}
				if (RobotContainer.manipController.getAButton()
						&& RobotContainer.isDriving().getAsBoolean()) {
					outakeSpeed = 4000; // maybe 4070 was outakeSpeed = 0.49 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 4000),-0.1,0.1);   
				} else if (RobotContainer.manipController.getXButton()
						&& RobotContainer.isDriving().getAsBoolean()) {
					outakeSpeed = 2700; // was outakeSpeed = 0.355 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 2700), -0.1, 0.1);
				} else {
					outakeSpeed = 0;
				}
			}
		}
		outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
