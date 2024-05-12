package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.SwerveS;
import frc.robot.utils.SimShootNote;
import frc.robot.DataHandler;

public class AutonIntake extends Command {
	private final IntakeS intakeS;
	private final SwerveS swerveS;
	private boolean isFinished = false;
	private boolean loaded = false;
	public static boolean allClear = false;
	public static boolean takeOver = false;
	private static boolean close = false, saver = false;
	private Translation2d targetNoteLocation = null;
	private Pose2d currentPose;
	private ChassisSpeeds speeds;
	double ty;
	Timer timer = new Timer();
	Timer delayTimer = new Timer();
	double desiredHeading, currentHeading, error;

	/*
	 * Call this in all cases but
	 * simulation autonomous
	 */
	public AutonIntake(IntakeS intakeS, SwerveS swerveS) {
		this.intakeS = intakeS;
		this.swerveS = swerveS;
		addRequirements(intakeS);
		this.targetNoteLocation = intakeS.getClosestNote();
	}

	/*
	 * Call this for simulation
	 * autonomous only
	 */
	public AutonIntake(IntakeS intakeS, SwerveS swerveS,
			Translation2d fieldNotePose) {
		this.intakeS = intakeS;
		this.swerveS = swerveS;
		this.targetNoteLocation = fieldNotePose;
	}

	@Override
	public void initialize() {
		if (Robot.isSimulation()) {
			this.targetNoteLocation = intakeS.getClosestNote();
			if (SimShootNote.currentNotes.get(0).getZ() > Units
					.inchesToMeters(1)) {
				saver = true;
			} else {
				saver = false;
			}
		}
		isFinished = false;
		LimelightHelpers.setPipelineIndex(
				Constants.LimelightConstants.limelightName, 1);
		delayTimer.reset();
		//delayTimer.start(); //for now
		allClear = false;
		close = false;
		takeOver = true;
		ty = 0;
		intakeS.deployIntake(intakeS.outsideBotState());
		IntakeS.autoIntakeController.reset();
		DataHandler.logData(new double[] {
				targetNoteLocation.getDistance(
						SwerveS.getPose().getTranslation()),
				IntakeS.kP
		});
	}

	public static double closerAngleToZero(double angle) {
		// Normalize the angle to be within the range of -180 to 180 degrees
		double normalizedAngle = angle % 360;
		if (normalizedAngle > 180) {
			normalizedAngle -= 360;
		} else if (normalizedAngle < -180) {
			normalizedAngle += 360;
		}
		// Return the closer angle to zero
		return (Math.abs(normalizedAngle) <= 180)
				? normalizedAngle
				: -normalizedAngle;
	}

	public static double speedMapper(double x) {
		// Define the parameters for the sigmoid function
		double x0 = 25; // Inches where the function starts to rise significantly
		double k = 0.1; // Steepness of the curve
		// Apply the sigmoid function to map x to the range [0, 1]
		double y = 1 / (1 + Math.exp(-k * (x - x0)));
		// Adjust the output to meet your specific points
		if (x >= 50) {
			y = 1;
		}
		return y;
	}

	@Override
	public void execute() {
		double tx, ty, distance = 0;
		boolean tv;
		if (Robot.isSimulation()) {
			//Computing distance
			currentPose = SwerveS.getPose();
			//THESE ARE IN M E T E R S 
			tx = targetNoteLocation.getX()
					- currentPose.getX();// -0.307975;
			ty = targetNoteLocation.getY()
					- currentPose.getY();// -0.307975;
			distance = Math
					.sqrt(Math.pow(tx, 2) + Math.pow(ty, 2));
			distance -= Constants.DriveConstants.kChassisLength;
			distance = Units.metersToInches(distance);
			tv = true;
			// Calculate the angle to aim towards the target
			double angleToTarget;
			if (tx == 0) {
				angleToTarget = 90;
			} else {
				angleToTarget = Math.atan2(ty, tx); // Calculate angle in radians
				angleToTarget = Math.toDegrees(angleToTarget);
			}
			// Convert angle to degrees if necessary
			// Adjust the robot's orientation towards the target
			// Example: Set the desired heading for your motion control system
			desiredHeading = angleToTarget;
			currentHeading = SwerveS.getHeading();
			error = -1 * (currentHeading - desiredHeading);
			error = closerAngleToZero(error);
		} else {
			IntakeS.detected = IntakeS.colorSensorV3
					.getColor();
			IntakeS.colorMatchResult = IntakeS.colorMatch
					.matchClosestColor(IntakeS.detected);
			//when done, set timer.start().. and delayTimer.stop();
			//THESE ARE IN D E G R E E S 
			tx = LimelightHelpers.getTX(
					Constants.LimelightConstants.limelightName);
			tv = LimelightHelpers.getTV(
					Constants.LimelightConstants.limelightName);
			ty = LimelightHelpers.getTY(
					Constants.LimelightConstants.limelightName);
			error = -tx; //tx is neg apparently -docs
		}
		SmartDashboard.putNumber("ANGLE ERROR", error);
		// SmartDashboard.putBoolean("Note Loaded?", IntakeS.noteIsLoaded());
		if (Robot.isReal()) {
			if (CameraS.calculateDistanceFromtY(ty) <= 4.5) { //less than twelve inches away, tune down!
				close = true;
			}
		} else {
			SmartDashboard.putNumber("SIMDISTANCE ",
					Units.metersToInches(distance));
			if (distance <= 4.5) {
				close = true;
			}
		}
		if (tv == false && loaded == false
				&& close == false) {
			// We don't see the target, seek for the target by spinning in place at a safe speed.
			speeds = new ChassisSpeeds(0, 0, 0.1
					* Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
		} else if (loaded == false && close == false) {
			double moveSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond
					* speedMapper(distance);
			double turnSpeed = Units.degreesToRadians(error)
					* IntakeS.kP;
			speeds = new ChassisSpeeds(moveSpeed, 0,
					turnSpeed);
		} else {
			speeds = new ChassisSpeeds(0, 0, 0);
		}
		swerveS.setChassisSpeeds(speeds);
		intakeS.setPrimaryIntake(-.75);
		//runs intake if note is not loaded
		if (Robot.isReal()) {
			if (IntakeS.noteIsLoaded()
					|| delayTimer.get() > 2) {
				delayTimer.stop();
				isFinished = true;
			}
		} else {
			if (close || delayTimer.get() > 2) {
				delayTimer.stop();
				isFinished = true;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		takeOver = false;
		intakeS.setPrimaryIntake(0);
		intakeS.deployIntake(intakeS.insideBotState());
		timer.stop();
		timer.reset();
		delayTimer.stop();
		delayTimer.reset();
		speeds = new ChassisSpeeds(0, 0, 0);
		swerveS.setChassisSpeeds(speeds);
		intakeS.pullBackNote();
		if (Robot.isSimulation() && close) {
			if (saver) {
				SimShootNote
						.intake(IntakeS.cloestNoteIndex - 1);
			} else {
				SimShootNote.intake(IntakeS.cloestNoteIndex);
			}
		}
		close = false;
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
