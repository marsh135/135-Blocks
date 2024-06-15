package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DrivetrainS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;

public class AimAtPoseC extends Command {
	private DrivetrainS drivetrainS;
	private Pose2d aimedPose;
	private PIDController drivePIDController;
	
	private boolean isFinished;
	private static final LoggableTunedNumber p = new LoggableTunedNumber(
			"AimAtPose P", 0.5);
	private static final LoggableTunedNumber i = new LoggableTunedNumber(
			"AimAtPose I", 0);
	private static final LoggableTunedNumber d = new LoggableTunedNumber(
			"AimAtPose D", 0);
	private static final double deadbandAngleDegrees = 2;
	private double angleDegrees;

	public AimAtPoseC(DrivetrainS drivetrain, Pose2d pose) {
		drivetrainS = drivetrain;
		aimedPose = pose;
		addRequirements(drivetrainS);
	}

	@Override
	public void initialize() {
		isFinished = false;
		Pose2d drivePose = drivetrainS.getPose();
		angleDegrees = Units
				.degreesToRadians(Math.atan((aimedPose.getY() - drivePose.getY())
						/ (aimedPose.getX() - drivePose.getX())));
		drivePIDController = new PIDController(p.get(), i.get(), d.get());
	}

	@Override
	public void execute() {
		if ((Math.abs(angleDegrees) - drivetrainS.getRotation2d()
				.getDegrees()) > deadbandAngleDegrees) {
			double omegaAngularVel = drivePIDController
					.calculate(deadbandAngleDegrees, angleDegrees);
			switch (DriveConstants.driveType) {
			// This means that it can't rotate and drive, so we can just set a chassisSpeeds with omega
			case TANK:
				drivetrainS
						.setChassisSpeeds(new ChassisSpeeds(0, 0, omegaAngularVel));
				break;
			default:

				break;
			}
		} else {
			isFinished = true;
		}
	}
	@Override
	public void end(boolean interrupted){

	}
	@Override 
	public boolean isFinished(){
		return isFinished;
	}
}
