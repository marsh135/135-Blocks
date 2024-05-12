package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autoCommands.AutonIntake;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.SwerveS;

/*
 * Use this as explanation on how to
 * pull DataLogs:
 * https://docs.wpilib.org/en/stable/
 * docs/software/telemetry/datalog-
 * download.html Should theoretically
 * output a csv file, see if we can
 * convert it into a txt file and upload
 * it to smth
 */
public class SwerveC extends Command {
	public ChassisSpeeds chassisSpeeds;
	private final SwerveS swerveS;
	private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
	public static double[][] variableAngleLog = new double[2][20];
	public static double variableAngleDistance = 0;
	public static double angleOutputDegrees = 0;
	//private int arrayIndex = 0;

	public SwerveC(SwerveS swerveS) {
		this.swerveS = swerveS;
		// These guys limit acceleration, they aren't the most necessary but it makes movement smoother
		this.xLimiter = new SlewRateLimiter(
				Constants.DriveConstants.kTeleDriveMaxAcceleration);
		this.yLimiter = new SlewRateLimiter(
				Constants.DriveConstants.kTeleDriveMaxAcceleration);
		this.turningLimiter = new SlewRateLimiter(
				Constants.DriveConstants.kTeleTurningMaxAcceleration);
		addRequirements(swerveS);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (!AutonIntake.takeOver) {
			// Get desired ChassisSpeeds from controller
			double xSpeed = -RobotContainer.driveController.getLeftY();
			double ySpeed = -RobotContainer.driveController.getLeftX();
			double turningSpeed = -RobotContainer.driveController.getRightX();
			xSpeed = Math.pow(xSpeed, 2) * (xSpeed < 0 ? -1 : 1);
			ySpeed = Math.pow(ySpeed, 2) * (ySpeed < 0 ? -1 : 1);
			//turningSpeed = Math.pow(turningSpeed, 2) * (turningSpeed < 0 ? -1 : 1);
			if (SwerveS.autoLock == true && CameraS.aprilTagVisible() == true) {
				turningSpeed = swerveS.autoLockController
						.calculate(CameraS.getXError(), 0.0);
			}
			//every time the array isn't full and the logging button is pressed, save another value to the array. 
			/*if (RobotContainer.driveController.getXButtonPressed() == true){
			  //angle is y, distance is x
			  try {
			 variableAngleLog[1][arrayIndex] = angleOutputDegrees;
			 variableAngleLog[0][arrayIndex] = variableAngleDistance;
			 System.out.println("Logged!");
			 arrayIndex +=1;
			  } catch (Exception e) {
			 System.out.println("Array Full!");
			 arrayIndex = 20;
			 printData();
			  }
			}
			
			if (RobotContainer.driveController.getBButtonPressed() == true){
			  //prints array and logs it in dataLog
			  printData();
			}*/
			// If the desired ChassisSpeeds are really small (ie from controller drift) make them even smaller so that the robot doesn't move
			xSpeed = Math.abs(xSpeed) > Constants.SwerveConstants.kDeadband
					? xSpeed
					: 0.0000;
			ySpeed = Math.abs(ySpeed) > Constants.SwerveConstants.kDeadband
					? ySpeed
					: 0.0000;
			if (SwerveS.autoLock == true && CameraS.aprilTagVisible() == true) {
				turningSpeed = Math
						.abs(turningSpeed) > Constants.SwerveConstants.kAutoDeadband
								? turningSpeed
								: 0.0000;
			} else {
				turningSpeed = Math
						.abs(turningSpeed) > Constants.SwerveConstants.kDeadband
								? turningSpeed
								: 0.0000;
			}
			// Limit the acceleration and convert -1 to 1 from the controller into actual speeds
			xSpeed = xLimiter.calculate(xSpeed)
					* Constants.DriveConstants.kMaxSpeedMetersPerSecond;
			ySpeed = yLimiter.calculate(ySpeed)
					* Constants.DriveConstants.kMaxSpeedMetersPerSecond;
			turningSpeed = turningLimiter.calculate(turningSpeed)
					* Constants.DriveConstants.kMaxTurningSpeedRadPerSec;
			if (SwerveS.getAlliance()) {
				xSpeed *= -1;
				ySpeed *= -1;
				turningSpeed *= 1;
			}
			// Convert ChassisSpeeds into the ChassisSpeeds type
			if (SwerveS.fieldOriented) {
				chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
						ySpeed, turningSpeed, SwerveS.getRotation2d());
			} else {
				chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
			}
			// Convert ChassisSpeeds into individual module states
			SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
					.toSwerveModuleStates(chassisSpeeds);
			// Set our module states to our desired module states
			swerveS.setModuleStates(moduleStates);
		}
	}

	/*Use this link to compute the regression model:https://planetcalc.com/5992/#google_vignette 
	 Each of the files has an x and y output so put those in the respective lists, or use a ti-84 stats bar*/
	/*public void printData() {
		//outputs collected distance vs angle graph to console and also sends it to the data logging file. 
		System.out.println("Distance (X)                              Angle (Y)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = " " + Double.toString(variableAngleLog[0][i]) + "    "
					+ Double.toString(variableAngleLog[1][i]);
			System.out.println(output);
			DataHandler
					.logData(new String[] { Double.toString(variableAngleLog[0][i]),
							Double.toString(variableAngleLog[1][i])
					});
		}
		System.out.println("Distance (X)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = Double.toString(variableAngleLog[0][i]) + " ";
			System.out.println(output);
		}
		System.out.println("Angle (Y)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = Double.toString(variableAngleLog[1][i]) + " ";
			System.out.println(output);
		}
		variableAngleLog = new double[2][20];
		arrayIndex = 0;
	} */

	@Override
	public void end(boolean interrupted) { swerveS.stopModules(); }

	@Override
	public boolean isFinished() { return false; }
}
