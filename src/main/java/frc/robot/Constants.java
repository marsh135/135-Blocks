// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	//TODO: Make a PID constants global. 
	public static class AutoConstants {
		public static HashMap<String, Command> eventMap = new HashMap<>();
	}

	public static final Mode currentMode = Mode.SIM;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,
		/** Running a physics simulator. */
		SIM,
		/** Replaying from a log file. */
		REPLAY
	}

	public static class IntakeConstants {
		public static class PIDConstants {
			public static double P = 8.0, //0.000001 
					I = 0.0, //.01
					D = 0.0001; //.000
		}

		public static class StateSpace {
			public static double kP = 00, kSVolts = 0, //must find
					kVVoltSecondsPerRotation = .0001, //must find
					kAVoltSecondsSquaredPerRotation = .0001; //must find
		}

		public static I2C.Port colorSensorPort = I2C.Port.kOnboard;
		public static Color noteColor = new Color(0.55, 0.36, .08);
		public static int primaryIntakeID = 20, deployIntakeID = 21,
				intakeAbsEncoderID = 1, intakeLimitSwitchID = 9, intakeOffset = 43;
		public static double absIntakeEncoderOffset = 59.870652,
				absIntakeEncoderConversionFactor = 360,
				primaryIntakeGearRatio = 1 / 4.5, deployIntakeInnerBound = 0,
				deployIntakeOuterBound = 90, macroMoveSpeed = .5;
		public static boolean primaryIntakeReversed = true,
				deployIntakeReversed = false;
	}

	public static class OutakeConstants {
		public static int topFlywheel = 30, bottomFlywheel = 31;
		public static double limelightToShooter = Units.inchesToMeters(-3),
				shooterHeight = Units.inchesToMeters(15), flywheelMaxRPM = 7100,
				flywheelGearRatio = 1.5, idealPercentTop = .34,
				idealPercentBottom = .31, 
				
				kFlywheelMomentOfInertia = .00032, //0.00006502025 //.001086 //.00066
				kP = 0.0021258, kSVolts = -0.089838,
				kVVoltSecondsPerRotation = 0.0015425 * .928,
				kAVoltSecondsSquaredPerRotation = 0.00039717 * 1;
		public static boolean topFlywheelReversed = false,
				bottomFlywheelReversed = false;
	}

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

				kFrontRightDriveP = 2.4646, //2.4646 maybe
				kFrontRightDriveSVolts = -0.040248,
				kFrontRightDriveVVoltSecondsPerRotation = 2.9041,
				kFrontRightDriveAVoltSecondsSquaredPerRotation = 1.52,
				
				kFrontLeftDriveP = 2.5896, //2.5896 //4.1054
				kFrontLeftDriveSVolts = -0.22934,
				kFrontLeftDriveVVoltSecondsPerRotation = 2.8559,
				kFrontLeftDriveAVoltSecondsSquaredPerRotation = 1.7338,

				kBackRightDriveP = 2.0873, //maybe 2.0873 or 1.4862 //4.1688
				kBackRightDriveSVolts = 0.070421,
				kBackRightDriveVVoltSecondsPerRotation = 2.8607,
				kBackRightDriveAVoltSecondsSquaredPerRotation = 1.1811,

				kBackLeftDriveP = 2.3375, //maybe 2.3375 or 1.5638 //3.9698
				kBackLeftDriveSVolts = 0.01842,
				kBackLeftDriveVVoltSecondsPerRotation = 2.7005,
				kBackLeftDriveAVoltSecondsSquaredPerRotation = 1.4511,
				//Window size is 1, velocity threshold is 75.
				//Motor ID 17 
				kFrontLeftTurnP = 1.0181, //maybe 0
				kFrontLeftTurnKs = 0.34809, kFrontLeftTurnKv = 0.0021885,
				kFrontLeftTurnKa = 0.00019056,
				//Motor ID 11
				kFrontRightTurnP = 0.99768, //maybe 0
				kFrontRightTurnKs = 0.28984, kFrontRightTurnKv = 0.0021057,
				kFrontRightTurnKa = 0.00018697,
				//Motor ID 15
				kBackLeftTurningP = 1.0521, kBackLeftTurningKs = 0.26615,
				kBackLeftTurningKv = 0.0021315, kBackLeftTurningKa = 0.00019805,
				//Motor ID 13
				kBackRightTurningP = 1.2362, kBackRightTurningKs = 0.25885,
				kBackRightTurningKv = 0.0021008, kBackRightTurningKa = 0.0002368;

		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		public static double[] overallTurnkPkSkVkAkD = new double[] {
				Constants.DriveConstants.kOverallPTurn * 4,
				Constants.DriveConstants.kOverallSVoltsTurn,
				Constants.DriveConstants.kOverallVVoltSecondsPerRotationTurn * 2,
				Constants.DriveConstants.kOverallAVoltSecondsSquaredPerRotationTurn,
				Constants.DriveConstants.kOverallDTurn * 1
		},
		overallDrivekPkSkVkA = new double[] {
				Constants.DriveConstants.kOverallP,
				Constants.DriveConstants.kOverallSVolts,
				Constants.DriveConstants.kOverallVVoltSecondsPerRotation,
				Constants.DriveConstants.kOverallAVoltSecondsSquaredPerRotation
		},
		frontRightDriveKpKsKvKa = new double[] { 2.4646, .04248, 2.9041, 1.52
		},
		frontLeftDriveKpKsKvKa = new double[] { 2.5896, .22934, 2.8559, 1.7338
		},
		backRightDriveKpKsKvKa = new double[] { 2.0873, .070421, 2.8607, 1.1811
		},
		backLeftDriveKpKsKvKa = new double[] { 2.3375, .01842, 2.7005, 1.4511
		},
		frontLeftTurnKpKsKvKa = new double[] { 1.0181, .34809, .0021885,.00019056
		},
		frontRightTurnKpKsKvKa = new double[] { .99768, .28984, .0021057, .00018697
		},
		backLeftTurnKpKsKvKa = new double[] { 1.0521, .26615, .26615, .0021315
		},
		backRightTurnKpKsKvKa = new double[] { 1.2362, .25885, .0021008, .0002368
		};
	}

	public static class DriveConstants {
		public static double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
				kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
				kDriveBaseRadius = Units
						.inchesToMeters(Math.sqrt(kChassisLength * kChassisLength
								+ kChassisWidth * kChassisWidth) / 2),
				// Distance from center of robot to the farthest module
				kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1),
				kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
				kTeleDriveMaxAcceleration = Units.feetToMeters(12), //guess
				kTeleTurningMaxAcceleration = 2 * Math.PI, //guess
				// To find these set them to zero, then turn the robot on and manually set the wheels straight.
				// The encoder values being read are then your new Offset values
				kFrontLeftAbsEncoderOffsetRad = 0.562867,
				kFrontRightAbsEncoderOffsetRad = 0.548137,
				kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
				kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861, kP = 0.1,
				kI = 0, kD = 0, kDistanceMultipler = .2;
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
		public static int kFrontLeftDrivePort = 16, //10
				kFrontLeftTurningPort = 17, //20
				kFrontLeftAbsEncoderPort = 2, //1
				kFrontRightDrivePort = 10, //11
				kFrontRightTurningPort = 11, //21
				kFrontRightAbsEncoderPort = 0, //2
				kBackLeftDrivePort = 14, //13
				kBackLeftTurningPort = 15, //23
				kBackLeftAbsEncoderPort = 3, //3
				kBackRightDrivePort = 12, //14
				kBackRightTurningPort = 13, //24
				kBackRightAbsEncoderPort = 1; //4
		public static boolean kFrontLeftDriveReversed = true,
				kFrontLeftTurningReversed = true,
				kFrontLeftAbsEncoderReversed = false,
				kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
				kFrontRightAbsEncoderReversed = false,
				kBackLeftDriveReversed = false, kBackLeftTurningReversed = true,
				kBackLeftAbsEncoderReversed = false,
				kBackRightDriveReversed = false, kBackRightTurningReversed = true,
				kBackRigthAbsEncoderReversed = false;
		//Unused
		public static double kOverallP = 2.36975, kOverallSVolts = -.180747,
				kOverallVVoltSecondsPerRotation = 2.8303,
				kOverallAVoltSecondsSquaredPerRotation = 1.4715,

				kFrontRightDriveP = 4.125, //2.4646 maybe
				kFrontRightDriveSVolts = -0.040248,
				kFrontRightDriveVVoltSecondsPerRotation = 2.9041,
				kFrontRightDriveAVoltSecondsSquaredPerRotation = 1.52,

				kFrontLeftDriveP = 4.1054, //2.5896 //4.1054
				kFrontLeftDriveSVolts = -0.22934,
				kFrontLeftDriveVVoltSecondsPerRotation = 2.8559,
				kFrontLeftDriveAVoltSecondsSquaredPerRotation = 1.7338,

				kBackRightDriveP = 4.1688, //maybe 2.0873 or 1.4862 //4.1688
				kBackRightDriveSVolts = 0.070421,
				kBackRightDriveVVoltSecondsPerRotation = 2.8607,
				kBackRightDriveAVoltSecondsSquaredPerRotation = 1.1811,

				kBackLeftDriveP = 3.9698, //maybe 2.3375 or 1.5638 //3.9698
				kBackLeftDriveSVolts = 0.01842,
				kBackLeftDriveVVoltSecondsPerRotation = 2.7005,
				kBackLeftDriveAVoltSecondsSquaredPerRotation = 1.4511,
				//Motor ID 17 
				kOverallSVoltsTurn = .2907325,
				kOverallVVoltSecondsPerRotationTurn = .002131625,
				kOverallAVoltSecondsSquaredPerRotationTurn = .000203095,
				kOverallPTurn = 1.07602, kOverallDTurn = 0.019508,
				kFrontLeftTurnP = 1.0181, //maybe 0
				kFrontLeftTurnD = 0.018265, kFrontLeftTurnKs = 0.34809,
				kFrontLeftTurnKv = 0.0021885, kFrontLeftTurnKa = 0.00019056,
				//Motor ID 11
				kFrontRightTurnP = 0.99768, //maybe 0
				kFrontRightTurnD = 0.017936, kFrontRightTurnKs = 0.28984,
				kFrontRightTurnKv = 0.0021057, kFrontRightTurnKa = 0.00018697,
				//Motor ID 15
				kBackLeftTurningP = 1.0521, kBackLeftTurningD = 0.019017,
				kBackLeftTurningKs = 0.26615, kBackLeftTurningKv = 0.0021315,
				kBackLeftTurningKa = 0.00019805,
				//Motor ID 13
				kBackRightTurningP = 1.2362, kBackRightTurningD = 0.022814,
				kBackRightTurningKs = 0.25885, kBackRightTurningKv = 0.0021008,
				kBackRightTurningKa = 0.0002368;
	}

	public static class DriveSimConstants {
		//id 1 is topmost leftmost. goes in order down, right.
		public static Pose3d[] fieldNoteTranslations = new Pose3d[] {
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(228),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center up 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(294),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center up 2
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(96),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center down 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(30),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center down 2
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BLUE CENTER
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BLUE CENTER + 1
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BUE TOP 
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED CENTER
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED CENTER + 1
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED TOP 
		};
	}

	public static class LEDConstants {
		//basically anything pertaining to the LEDs go in here
		public static int
		//ledPort
		ledPort = 9,
				// amount of LEDs in the light strip
				ledBufferLength = 90, sineWaveUpdateCycles = 1,
				overrideLEDPatternTime = 4;
		//all arrays below use the H,S,V format
		public static int[] noteHSV = new int[] { 12, 255, 100
		}, redHSV = new int[] { 0, 255, 100
		}, blueHSV = new int[] { 120, 255, 100
		}, greenHSV = new int[] { 50, 255, 100
		}, pinkHSV = new int[] { 147, 255, 255
		}, goldHSV = new int[] { 23, 255, 100
		}, disabledHSV = new int[] { 0, 0, 0
		};
		public static int sinePeriod = 32;
		public static int[] ledStates = new int[LEDConstants.sinePeriod];
		//Basically controls how different the waves are from one another when the setColorWave function is called. Due to how it is calculated , this value CANNOT be zero (divide by zero error). 
		//Try to set this value to a multiple of however many LEDs we have (so like if we have 63 LEDs on the robot set the sine to 9)
		//goes half as fast in idle
	}

	public static class HangConstants {
		public static double upperHookHeight = 63, hangLowerSoftStop = 5,
				hangUpperSoftStop = 101; //Note: for some reason left and right encoders output different values, MAYBE change them to have left and right max?
		//left side is left, right side is right
		public static int leftHangID = 41, rightHangID = 40;
		public static boolean leftHangReversed = true, rightHangReversed = false;
	}

	public static class LimelightConstants {
		public static double limeLightAngleOffsetDegrees = -30,
				limelightLensHeightoffFloorInches = 22.5;
		public static String limelightName = "limelight-swerve";
	}

	public static class VisionConstants {
		//These are all placeholders
		//Camera names, from photonVision web interface
		public static String frontCamName = "Front_Camera",
				backCamName = "Back_Camera", leftCamName = "Left_Camera",
				rightCamName = "Right_Camera";
		public static String[] camNameStrings = new String[] { frontCamName,
				backCamName, leftCamName, rightCamName
		};
		//offset of each cam from robot center, in meters
		public static Translation3d frontCamTranslation3d = new Translation3d(
				Units.inchesToMeters(15), 0, Units.inchesToMeters(19.75)),
				rightCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75),
						Units.inchesToMeters(
								Constants.DriveConstants.kChassisLength / 2),
						Units.inchesToMeters(19.75)),
				/*For right cam ^
				  X = center to limelight tube
				  Y = center to right chassis rail
				  Z = floor to camera 
				*/
				leftCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75),
						Units.inchesToMeters(
								-Constants.DriveConstants.kChassisLength / 2),
						Units.inchesToMeters(19.75)),
				backCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75), Units.inchesToMeters(0),
						Units.inchesToMeters(25));
		//Pitches of camera, in DEGREES, positive means UPWARD angle
		public static int frontCamPitch = -21, rightCamPitch = -21, //MUST GET
				leftCamPitch = -15, backCamPitch = -26, camResWidth = 600,
				camResHeight = 800, camFPS = 60;
		public static int[] camPitches = new int[] { frontCamPitch, rightCamPitch,
				leftCamPitch, backCamPitch
		};
		public static Rotation2d camFOV = new Rotation2d(
				Units.degreesToRadians(62.5));
		public static int[] camAvgError = new int[] { 0, 0, 0, 0
		}, camAvgStdDev = new int[] { 0, 0, 0, 0
		}, camAvgLatencyMs = new int[] { 10, 10, 10, 10
		}, camAvgLatencyStdDev = new int[] { 5, 5, 5, 5
		};
	}

	public static class FieldConstants {
		public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,
				4, 8);
		public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField
				.loadAprilTagLayoutField();
		public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,
				0.5, 1);
		public static double targetHeightoffFloorInches = 57,
				speakerLowerLipHeight = Units.inchesToMeters(78.13),
				speakerUpperLipHeight = Units.inchesToMeters(80.9),
				noteHeight = Units.inchesToMeters(2.5),
				speakerOpeningDepth = Units.inchesToMeters(18.11);
		public static Translation2d blueSpeaker = new Translation2d(0, 5.56),
				redSpeaker = new Translation2d(16.59128, 5.56);
	}

	public static class DataLog {
		public static Map<Integer, String> manCanIdsToNames() {
			HashMap<Integer, String> map = new HashMap<>();
			map.put(Constants.OutakeConstants.topFlywheel, "topFlywheel");
			return map;
		}

		public static int testRunSeconds = 10;
		public static double[][] variableAngleLog = new double[2][20];
		public static double variableAngleDistance = 0;
		public static double angleOutputDegrees = 0;
	}
}