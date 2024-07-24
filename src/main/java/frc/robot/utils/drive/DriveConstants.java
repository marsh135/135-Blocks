package frc.robot.utils.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.MotorConstantContainer;

public class DriveConstants {
	public static MotorVendor robotMotorController = MotorVendor.CTRE_MOTORS;
	public static DriveTrainType driveType = DriveTrainType.SWERVE;
	public static GyroType gyroType = GyroType.PIGEON;

	/**
	 * What motors and motorContollers are we using
	 */
	public enum MotorVendor {
		NEO_SPARK_MAX, VORTEX_SPARK_FLEX, CTRE_MOTORS
	}

	/**
	 * The drivetrain type
	 */
	public enum DriveTrainType {
		SWERVE, TANK, MECANUM
	}

	/**
	 * The Gyro type
	 * 
	 * @apiNote NavX Swerve is untested.
	 */
	public enum GyroType {
		NAVX, PIGEON
	}

	public static boolean fieldOriented = true;
	//135-Blocks was tested on a chassis with all CANSparkMaxes, as well as all Kraken-x60s.
	public static final double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
			kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
			kDriveBaseRadius = Math.sqrt(
					kChassisLength * kChassisLength + kChassisWidth * kChassisWidth)
					/ 2,
			// Distance from center of robot to the farthest module
			kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1), //15.1
			kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
			kTeleDriveMaxAcceleration = Units.feetToMeters(50), // guess
			kTeleTurningMaxAcceleration = 2 * Math.PI * 50, // guess
			// To find these set them to zero, then turn the robot on and manually set the
			// wheels straight.
			// The encoder values being read are then your new Offset values
			//For REV
			/*kFrontLeftAbsEncoderOffsetRad = 0.562867,
			kFrontRightAbsEncoderOffsetRad = 0.548137,
			kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
			kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861,*/
			//For CTRE
			kFrontLeftAbsEncoderOffsetRad = -1.176212,
			kFrontRightAbsEncoderOffsetRad = 1.580000,
			kBackLeftAbsEncoderOffsetRad = -2.86547,
			kBackRightAbsEncoderOffsetRad = -.031415,

			SKID_THRESHOLD = .5, //Meters per second
			MAX_G = .5;
	public static PathConstraints pathConstraints = new PathConstraints(
			kMaxSpeedMetersPerSecond, kTeleDriveMaxAcceleration,
			kMaxTurningSpeedRadPerSec, kTeleTurningMaxAcceleration);
	// kP = 0.1, kI = 0, kD = 0, kDistanceMultipler = .2; //for autoLock
	// Declare the position of each module
	public static final Translation2d[] kModuleTranslations = {
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2)
	};

	public static int 
			kFrontLeftDrivePort = 16, 
			kFrontLeftTurningPort = 17, 
			kFrontLeftAbsEncoderPort = 20, 
			kFrontRightDrivePort = 10, 
			kFrontRightTurningPort = 11, 
			kFrontRightAbsEncoderPort = 21, 
			kBackLeftDrivePort = 14, 
			kBackLeftTurningPort = 15, 
			kBackLeftAbsEncoderPort = 23, 
			kBackRightDrivePort = 12, 
			kBackRightTurningPort = 13, 
			kBackRightAbsEncoderPort = 24, 
			kMaxDriveCurrent = 40, kMaxTurnCurrent = 30;
	public static boolean kFrontLeftDriveReversed = true,
			kFrontLeftTurningReversed = true, kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
			kFrontRightAbsEncoderReversed = false, kBackLeftDriveReversed = false,
			kBackLeftTurningReversed = true, kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = false, kBackRightTurningReversed = true,
			kBackRightAbsEncoderReversed = false, kGyroReversed = true;
	public static final ModuleLimits moduleLimitsFree = new ModuleLimits(
			DriveConstants.kMaxSpeedMetersPerSecond,
			DriveConstants.kTeleDriveMaxAcceleration,
			Units.degreesToRadians(1080.0));

	public static class TrainConstants {
		/**
		 * Which swerve module it is (SWERVE EXCLUSIVE)
		 */
		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(
				VecBuilder.fill(0.003, 0.003, 0.0002));
		public static double kWheelDiameter = Units.inchesToMeters(3.873),
				kDriveMotorGearRatio = 6.75, kTurningMotorGearRatio = 150 / 7,
				kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI, //Test if wheelDiameter should be here..?
				kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60,
				kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI,
				kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60,
				kT = 1.0 / DCMotor.getNEO(1).KtNMPerAmp, kDeadband = 0.05,
				weight = Units.lbsToKilograms(110);
		public static final MotorConstantContainer overallTurningMotorConstantContainer = new MotorConstantContainer(
				0.001, 0.001, 0.001, 5, 0.001), //Average the turning motors for these vals.
				overallDriveMotorConstantContainer = new MotorConstantContainer(.1,
						.13, 0.001, 0.05, 0.000);
	}
}
