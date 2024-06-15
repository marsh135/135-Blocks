package frc.robot.commands.drive;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

public class AimAtPoseWhilePathing extends Command {
	private DrivetrainS drivetrainS;
	private Pose2d aimedPose;
	private boolean isFinished;
	public AimAtPoseWhilePathing(DrivetrainS drivetrain, Pose2d pose){
		drivetrainS = drivetrain;
		aimedPose = pose;
		addRequirements(drivetrainS);
	}
	@Override
	public void initialize(){
		isFinished = false;
	}
	@Override 
	public void execute(){
		Pose2d pose = drivetrainS.getPose();
		/*We do arctan here to compute the ideal theta. 
		arctan here should be change in x over change in y*/
		double desAngleRadians = Math.atan((aimedPose.getY()-pose.getY())/(aimedPose.getX()-pose.getX()));
		Rotation2d desiredRotation2d = Rotation2d.fromRadians(desAngleRadians);
		switch (DriveConstants.driveType) {
			case TANK:
				System.err.println("No support for AimAtPoseWhilePathing for tank!");
				break;	
			default:
				RobotContainer.angleOverrider = () -> {return Optional.of(desiredRotation2d);};
				break;
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
