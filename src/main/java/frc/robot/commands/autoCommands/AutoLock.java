package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AutoLock extends Command {
	SwerveS swerveS;
	boolean isFinished = false;
	double limelightDeadBand = 0.00001, limelightTx = 0,
			maxTimeTargetting = 0.1;
	Timer timer = new Timer();
	ChassisSpeeds speeds;

	public AutoLock(SwerveS swerveS) {
		this.swerveS = swerveS;
		addRequirements(swerveS);
	}

	@Override
	public void initialize() {
		isFinished = false;
		timer.stop();
		timer.reset();
	}

	@Override
	public void execute() {
		if (timer.get() >= maxTimeTargetting) {
			isFinished = true;
		}
		if (!CameraS.aprilTagVisible()) {
			timer.start();
		}
		//takes x value of limelight as the distance that needs to be rotated, runs a chassisSpeedscommand to rotate until within an acceptable deadband
		limelightTx = CameraS.getXError();
		SmartDashboard.putNumber("TX", limelightTx);
		if (Math.abs(limelightTx) < limelightDeadBand) {
			isFinished = true;
		}
		speeds = new ChassisSpeeds(0, 0,
				swerveS.autoLockController.calculate(limelightTx, 0)
						* Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
		swerveS.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
		speeds = new ChassisSpeeds(0, 0, 0);
		swerveS.setChassisSpeeds(speeds);
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
