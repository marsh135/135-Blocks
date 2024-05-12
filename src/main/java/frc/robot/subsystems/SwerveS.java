package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SimShootNote;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveS extends SubsystemBase {
	private Supplier<Pose2d> pose2dSupplier = () -> {
		return getPose();
	};
	private final static HashMap<ModulePosition, SwerveModule> m_swerveModules = new HashMap<>(
			Map.of(ModulePosition.FRONT_LEFT,
					new SwerveModule(Constants.DriveConstants.kFrontLeftDrivePort,
							Constants.DriveConstants.kFrontLeftTurningPort,
							Constants.DriveConstants.kFrontLeftDriveReversed,
							Constants.DriveConstants.kFrontLeftTurningReversed,
							Constants.DriveConstants.kFrontLeftAbsEncoderPort,
							Constants.DriveConstants.kFrontLeftAbsEncoderOffsetRad,
							Constants.DriveConstants.kFrontLeftAbsEncoderReversed,
							Constants.SwerveConstants.frontLeftDriveKpKsKvKa,
							Constants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.FRONT_RIGHT,
					new SwerveModule(Constants.DriveConstants.kFrontRightDrivePort,
							Constants.DriveConstants.kFrontRightTurningPort,
							Constants.DriveConstants.kFrontRightDriveReversed,
							Constants.DriveConstants.kFrontRightTurningReversed,
							Constants.DriveConstants.kFrontRightAbsEncoderPort,
							Constants.DriveConstants.kFrontRightAbsEncoderOffsetRad,
							Constants.DriveConstants.kFrontRightAbsEncoderReversed,
							Constants.SwerveConstants.frontRightDriveKpKsKvKa,
							Constants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.BACK_LEFT,
					new SwerveModule(Constants.DriveConstants.kBackLeftDrivePort,
							Constants.DriveConstants.kBackLeftTurningPort,
							Constants.DriveConstants.kBackLeftDriveReversed,
							Constants.DriveConstants.kBackLeftTurningReversed,
							Constants.DriveConstants.kBackLeftAbsEncoderPort,
							Constants.DriveConstants.kBackLeftAbsEncoderOffsetRad,
							Constants.DriveConstants.kBackLeftAbsEncoderReversed,
							Constants.SwerveConstants.backLeftDriveKpKsKvKa,
							Constants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.BACK_RIGHT,
					new SwerveModule(Constants.DriveConstants.kBackRightDrivePort,
							Constants.DriveConstants.kBackRightTurningPort,
							Constants.DriveConstants.kBackRightDriveReversed,
							Constants.DriveConstants.kBackRightTurningReversed,
							Constants.DriveConstants.kBackRightAbsEncoderPort,
							Constants.DriveConstants.kBackRightAbsEncoderOffsetRad,
							Constants.DriveConstants.kBackRightDriveReversed,
							Constants.SwerveConstants.backRightDriveKpKsKvKa,
							Constants.SwerveConstants.overallTurnkPkSkVkAkD)));
	private static AHRS gyro = new AHRS(Port.kUSB1);
	//NetworkTableEntry pipeline;
	public static boolean fieldOriented = true;
	int periodicUpdateCycle;
	/*public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.limelightName);
	static NetworkTableEntry tx = limelight.getEntry("tx");
	static double xError = tx.getDouble(0.0);*/
	static Pose2d robotPosition = new Pose2d(0, 0, getRotation2d());
	Field2d robotField = new Field2d();
	//static double zAccel = 0;
	// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
	ChassisSpeeds m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics
			.toChassisSpeeds(getModuleStates());
	static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
	static Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);
	static SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			Constants.DriveConstants.kDriveKinematics, getRotation2d(),
			getModulePositions(), robotPosition, stateStdDevs, visionStdDevs);
			
	SwerveModulePosition[] m_modulePositions = getModulePositions();
	private double m_simYaw;

	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(8);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState", state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> 
			{
				for (SwerveModule module : m_swerveModules.values())
					module.setTurningTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
			, this));

	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState", state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> 
			{
				for (SwerveModule module : m_swerveModules.values())
					module.setDriveTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
			, this));

	/**
	 * Returns a command that will execute a quasistatic test in the given
	 * direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	public Command sysIdQuasistaticTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.quasistatic(direction);
	}

	/**
	 * Returns a command that will execute a dynamic test in the given direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	public Command sysIdDynamicTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.dynamic(direction);
	}

	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.quasistatic(direction);
	}

	public static boolean autoLock = false;
	public static boolean redIsAlliance = true;
	private static double kP, kI, kD;
	public PIDController autoLockController; //sadly cannot be system Id'd
	//so that the navXDisconnect command doesn't start twice
	int debounce = 0;

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
		};
	}

	public static SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
		};
	}

	public SwerveS() {
		// Waits for the RIO to finishing booting
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				zeroHeading();
				for (SwerveModule module : m_swerveModules.values())
					module.resetEncoders();
				//limelight.getEntry("pipeline").setNumber(1);
			}
			catch (Exception e) {
			}
		}).start();
		AutoBuilder.configureHolonomic(SwerveS::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						Constants.DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), SwerveS::getAlliance, this // Reference to this subsystem to set requirements
		);
		//SmartDashboard.putData("Field", robotField);
		kP = Constants.DriveConstants.kP;
		kI = Constants.DriveConstants.kI;
		kD = Constants.DriveConstants.kD;
		SmartDashboard.putNumber("P Gain AutoLock", kP);
		SmartDashboard.putNumber("I Gain AutoLock", kI);
		SmartDashboard.putNumber("D Gain AutoLock", kD);
		autoLockController = new PIDController(kP, kI, kD);
		if (Robot.isSimulation()){
			SimShootNote.setRobotPoseSupplier(pose2dSupplier);
		}
	}

	public void zeroHeading() {
		debounce = 0;
		gyro.reset();
	}

	public static double getHeading() {
		return -1 * Math
				.IEEEremainder(gyro.getAngle() + (getAlliance() ? 180 : 0), 360); //modulus
	}

	public static Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}

	public static boolean getAutoLock() { return autoLock; }

	@Override
	public void periodic() {
		if (Robot.isSimulation()) {
			/*frontLeft.updateSimModuleState();
			frontRight.updateSimModuleState();
			backLeft.updateSimModuleState();
			backRight.updateSimModuleState();
			SmartDashboard.putNumber("Sim debug chassis x speed", m_ChassisSpeeds.vyMetersPerSecond);*/
			//LimelightSim.updateTarget(getPose());
			ChassisSpeeds chassisSpeed = Constants.DriveConstants.kDriveKinematics
					.toChassisSpeeds(getModuleStates());
			m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
			//m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
			SimShootNote.updateStates();
		}
		periodicUpdateCycle += 1;
		/*if (limelight.getEntry("pipeline").getDouble(0) != 1) {
		    limelight.getEntry("pipeline").setNumber(1);
		}
		
		if (getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 4.0) {
		    limelight.getEntry("priorityid").setNumber(4);
		} else if (!getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 7.0) {
		    limelight.getEntry("priorityid").setNumber(7);
		}*/
		/*if(periodicUpdateCycle%5 == 0){
		    updatePoseEstimatorWithVisionBotPose();
		}*/
		redIsAlliance = getAlliance();
		//zAccel = gyro.getRawAccelZ();
		//puts values to smartDashboard
		SmartDashboard.putNumber("xError", CameraS.backCamXError);
		SmartDashboard.putBoolean("Auto Lock", autoLock);
		SmartDashboard.putNumber("Robot Heading (getPose)",
				getPose().getRotation().getDegrees());
		double p = SmartDashboard.getNumber("P Gain AutoLock",
				Constants.DriveConstants.kP);
		double i = SmartDashboard.getNumber("I Gain AutoLock",
				Constants.DriveConstants.kI);
		double d = SmartDashboard.getNumber("D Gain AutoLock",
				Constants.DriveConstants.kD);
		if ((p != kP)) {
			autoLockController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			autoLockController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			autoLockController.setD(d);
			kD = d;
		}
		//xError = tx.getDouble(0.0);
		m_modulePositions = getModulePositions();
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics
				.toChassisSpeeds(getModuleStates());
		robotPosition = poseEstimator.update(getRotation2d(), m_modulePositions);
		for (SwerveModule module : m_swerveModules.values()) {
			var modulePositionFromChassis = Constants.DriveConstants.kModuleTranslations[module
					.getModuleNumber()].rotateBy(getRotation2d())
							.plus(getPoseMeters().getTranslation());
			module.setModulePose(new Pose2d(modulePositionFromChassis,
					module.getHeadingRotation2d().plus(getRotation2d())));
		}
		robotField.setRobotPose(getPose());
		SmartDashboard.putData(robotField);
		//SmartDashboard.putData(CameraS.getEstimatedField());
		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/CurrentPose", pose);
			robotField.setRobotPose(pose);
		});
		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
			robotField.getObject("target pose").setPose(pose);
		});
		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			Logger.recordOutput("Odometry/Trajectory",
					poses.toArray(new Pose2d[poses.size()]));
			robotField.getObject("path").setPoses(poses);
		});
		navXDisconnectProtocol();
		Logger.recordOutput("Swerve/Display/Actual Swerve Module States",
				getModuleStates());
		Logger.recordOutput("Swerve/Display/Rotation",
				getRotation2d().getDegrees());
	}
	/*public static double getZAccel(){
	    return zAccel;
	}*/
	//this is pitch owyn was tweaking
	/*public static double getZDistance(){
	    return limelight.getEntry("ty").getDouble(0.0);
	}*/
	/*public static double getXError() {
	    // bounds xError between -5 and 5 (normal range of xError is -30 to 30)
	    double bounded = xError/6 + Math.copySign(0.9999, xError); //adds 0.9999 to reduce dead area range once we square
	    return bounded*Math.abs(bounded);
	
	
	}*/

	/*public static boolean aprilTagVisible() {
	    return xError != 0.0;
	}*/
	@AutoLogOutput(key = "Odometry/Robot")
	public static Pose2d getPose() { return robotPosition; }

	public ChassisSpeeds getChassisSpeeds() { return m_ChassisSpeeds; }

	/**
	 * Returns whether the alliance is red.
	 */
	public static boolean getAlliance() {
		// Boolean supplier that controls when the path will be mirrored for the red alliance
		// This will flip the path being followed to the red side of the field.
		// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}

	public void resetPose(Pose2d pose) {
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		poseEstimator.resetPosition(getRotation2d(), m_modulePositions, pose);
	}

	public void stopModules() {
		for (SwerveModule module : m_swerveModules.values())
			module.stop();
	}

	public void toggleAutoLock() {
		autoLockController.reset();
		autoLock = !autoLock;
	}
	/*public void updatePoseEstimatorWithVisionBotPose() {
	   //sanity check, doesn't do anything if unexpected value occurs
	   
	   //computes latency
	   
	   poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.limelightName);
	   int count = poseEstimate.tagCount;
	   Pose2d poseLimelight = poseEstimate.pose;
	   double latency = Timer.getFPGATimestamp() - (limelight.getEntry("tl").getDouble(0.0)/1000.0) - (limelight.getEntry("cl").getDouble(0.0)/1000.0);
	   Pose2d odomPose = getPose();
	   SmartDashboard.putNumber("limelightx", poseLimelight.getX());
	   SmartDashboard.putNumber("limelight y", poseLimelight.getY());
	   Translation2d transOdom = new Translation2d(odomPose.getX(),odomPose.getY());
	   Translation2d transLim = new Translation2d(poseLimelight.getX(),poseLimelight.getY());
	   double poseDifference = transOdom.getDistance(transLim);
	   //ends if unreasonable result
	   if (poseLimelight.getX() == 0){
	       return;
	   }
	   if (count != 0){
	       double xyStds;
	       double degStds;
	       if (count>=2){
	           if (periodicUpdateCycle % 10 == 0 && !DriverStation.isAutonomous()) {
	               resetPose(poseLimelight);
	               return;
	           } else {
	               xyStds = 0.05;
	               degStds = 6;
	           }
	       }
	       if (poseEstimate.avgTagArea > .8 && poseDifference <.5){
	           xyStds = 1.0;
	           degStds = 12;
	       }
	       else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
	           xyStds = 2.0;
	           degStds = 30;
	       } else {
	           return;
	       }
	       poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
	       poseEstimator.addVisionMeasurement(poseLimelight, latency);
	   }
	   else{
	       return;
	   }
	  
	   
	
	   }*/

	public InstantCommand toggleAutoLockCommand() {
		return new InstantCommand(this::toggleAutoLock, this);
	}

	public Pose2d getPoseMeters() {
		return poseEstimator.getEstimatedPosition();
	}

	public void setChassisSpeeds(ChassisSpeeds speed) {
		SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
		for (SwerveModule module : m_swerveModules.values()) {
			module.setDesiredState(moduleStates[module.getModuleNumber()]);
		}
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		// Proportionally lowers wheel speeds until they are under the max speed
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		for (SwerveModule module : m_swerveModules.values()) {
			module.setDesiredState(desiredStates[module.getModuleNumber()]);
		}
		Logger.recordOutput("Swerve/Display/Target Swerve Module States",
				desiredStates);
	}

	/**
	 * Function that sets the robot to robot relative (and then sets the leds to
	 * a pattern showing that the navx has disconnected) if the navx has
	 * disconnected
	 */
	public void navXDisconnectProtocol() {
		if (gyro.isConnected() && debounce == 0) {
			fieldOriented = true;
			return;
		} else {
			debounce = -1;
			fieldOriented = false;
		}
	}

	public static void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}

	public static void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds,
				stdDevs);
	}
}
