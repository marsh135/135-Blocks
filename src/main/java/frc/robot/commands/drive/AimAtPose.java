package frc.robot.commands.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainS;

public class AimAtPose extends Command {
	DrivetrainS drivetrainS;
	Pose2d aimedPose;
	boolean isFinished;
	public AimAtPose(DrivetrainS drivetrain, Pose2d pose){
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
		/*We do arctan here to compute the ideal theta. 
		arctan here should be change in x over change in y*/
	}
}
