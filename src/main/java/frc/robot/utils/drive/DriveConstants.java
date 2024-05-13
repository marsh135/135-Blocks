package frc.robot.utils.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
	public static double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
			kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
			kDriveBaseRadius = Units
					.inchesToMeters(Math.sqrt(kChassisLength * kChassisLength
							+ kChassisWidth * kChassisWidth) / 2),
			// Distance from center of robot to the farthest module
			kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1),
			kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
			kTeleDriveMaxAcceleration = Units.feetToMeters(12), // guess
			kTeleTurningMaxAcceleration = 2 * Math.PI, // guess
			// To find these set them to zero, then turn the robot on and manually set the
			// wheels straight.
			// The encoder values being read are then your new Offset values
			kFrontLeftAbsEncoderOffsetRad = 0.562867,
			kFrontRightAbsEncoderOffsetRad = 0.548137,
			kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
			kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861;
	// kP = 0.1, kI = 0, kD = 0, kDistanceMultipler = .2; //for autoLock
	// Declare the position of each module
	public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2));
	public static final Translation2d[] kModuleTranslations = {
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2)
	};
	public static int kFrontLeftDrivePort = 16, // 10
			kFrontLeftTurningPort = 17, // 20
			kFrontLeftAbsEncoderPort = 2, // 1
			kFrontRightDrivePort = 10, // 11
			kFrontRightTurningPort = 11, // 21
			kFrontRightAbsEncoderPort = 0, // 2
			kBackLeftDrivePort = 14, // 13
			kBackLeftTurningPort = 15, // 23
			kBackLeftAbsEncoderPort = 3, // 3
			kBackRightDrivePort = 12, // 14
			kBackRightTurningPort = 13, // 24
			kBackRightAbsEncoderPort = 1; // 4
	public static boolean kFrontLeftDriveReversed = true,
			kFrontLeftTurningReversed = true,
			kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
			kFrontRightAbsEncoderReversed = false,
			kBackLeftDriveReversed = false, kBackLeftTurningReversed = true,
			kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = false, kBackRightTurningReversed = true,
			kBackRigthAbsEncoderReversed = false;
		
	public static class SwerveConstants {
		public static double kWheelDiameter = Units.inchesToMeters(3.873),
				kDriveMotorGearRatio = 1 / 6.75, kTurningMotorGearRatio = (7 / 150),
				kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
						* kWheelDiameter,
				kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60,
				kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI,
				kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60,
				kDeadband = 0.1, kAutoDeadband = 0.0000001, kOverallP = 2.36975,
				kOverallSVolts = -.180747, kOverallVVoltSecondsPerRotation = 2.8303,
				kOverallAVoltSecondsSquaredPerRotation = 1.4715,

				kFrontRightDriveP = 2.4646, // 2.4646 maybe
				kFrontRightDriveSVolts = -0.040248,
				kFrontRightDriveVVoltSecondsPerRotation = 2.9041,
				kFrontRightDriveAVoltSecondsSquaredPerRotation = 1.52,

				kFrontLeftDriveP = 2.5896, // 2.5896 //4.1054
				kFrontLeftDriveSVolts = -0.22934,
				kFrontLeftDriveVVoltSecondsPerRotation = 2.8559,
				kFrontLeftDriveAVoltSecondsSquaredPerRotation = 1.7338,

				kBackRightDriveP = 2.0873, // maybe 2.0873 or 1.4862 //4.1688
				kBackRightDriveSVolts = 0.070421,
				kBackRightDriveVVoltSecondsPerRotation = 2.8607,
				kBackRightDriveAVoltSecondsSquaredPerRotation = 1.1811,

				kBackLeftDriveP = 2.3375, // maybe 2.3375 or 1.5638 //3.9698
				kBackLeftDriveSVolts = 0.01842,
				kBackLeftDriveVVoltSecondsPerRotation = 2.7005,
				kBackLeftDriveAVoltSecondsSquaredPerRotation = 1.4511,
				// Window size is 1, velocity threshold is 75.
				// Motor ID 17
				kFrontLeftTurnP = 1.0181, // maybe 0
				kFrontLeftTurnKs = 0.34809, kFrontLeftTurnKv = 0.0021885,
				kFrontLeftTurnKa = 0.00019056,
				// Motor ID 11
				kFrontRightTurnP = 0.99768, // maybe 0
				kFrontRightTurnKs = 0.28984, kFrontRightTurnKv = 0.0021057,
				kFrontRightTurnKa = 0.00018697,
				// Motor ID 15
				kBackLeftTurningP = 1.0521, kBackLeftTurningKs = 0.26615,
				kBackLeftTurningKv = 0.0021315, kBackLeftTurningKa = 0.00019805,
				// Motor ID 13
				kBackRightTurningP = 1.2362, kBackRightTurningKs = 0.25885,
				kBackRightTurningKv = 0.0021008, kBackRightTurningKa = 0.0002368;

		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		public static double[] overallTurnkPkSkVkAkD = new double[] {
				1.07062 * 4,
				.2907325,
				.002131625,
				.000203095,
				0.019508
		},
				overallDrivekPkSkVkA = new double[] {
						2.36975,
						-.180747,
						2.8303,
						1.4715,
				},
				frontRightDriveKpKsKvKa = new double[] { 2.4646, .04248, 2.9041, 1.52
				},
				frontLeftDriveKpKsKvKa = new double[] { 2.5896, .22934, 2.8559, 1.7338
				},
				backRightDriveKpKsKvKa = new double[] { 2.0873, .070421, 2.8607, 1.1811
				},
				backLeftDriveKpKsKvKa = new double[] { 2.3375, .01842, 2.7005, 1.4511
				},
				frontLeftTurnKpKsKvKa = new double[] { 1.0181, .34809, .0021885, .00019056
				},
				frontRightTurnKpKsKvKa = new double[] { .99768, .28984, .0021057, .00018697
				},
				backLeftTurnKpKsKvKa = new double[] { 1.0521, .26615, .26615, .0021315
				},
				backRightTurnKpKsKvKa = new double[] { 1.2362, .25885, .0021008, .0002368
				};
	}

	public static class DriveSimConstants {
		// must be changed to new game piece locations!
		public static Pose3d[] fieldNoteTranslations = new Pose3d[] {
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(228),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center up 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(294),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center up 2
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(96),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center down 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(30),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center down 2
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BLUE CENTER
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BLUE CENTER + 1
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BUE TOP
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED CENTER
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED CENTER + 1
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED TOP
		};
	}
}
