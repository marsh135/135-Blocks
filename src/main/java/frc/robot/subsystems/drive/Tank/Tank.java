// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.Position;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Tank extends SubsystemChecker implements DrivetrainS {
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter
			/ 2;
	public static final double TRACK_WIDTH = DriveConstants.kChassisWidth;
	private final TankIO io;
	private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();
	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
			TRACK_WIDTH);
	private final SimpleMotorFeedforward feedforward = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getFeedforward();
	private final SysIdRoutine sysId;
	private DifferentialDrivePoseEstimator poseEstimator;
	private Twist2d fieldVelocity;
	private Rotation2d rawGyroRotation = new Rotation2d();
	private Pose2d pose = new Pose2d();
	private Position<DifferentialDriveWheelPositions> wheelPositions;
	private boolean collisionDetected;
	private int debounce = 0;

	/** Creates a new Drive. */
	public Tank(TankIO io) {
		this.io = io;
		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configureRamsete(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds,
				new ReplanningConfig(true, true), () -> Robot.isRed, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		// Configure SysId
		Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
		Measure<Voltage> holdVoltage = Volts.of(4);
		Measure<Time> timeout = Seconds.of(10);
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
						(state) -> Logger.recordOutput("Drive/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism(
						(voltage) -> driveVolts(voltage.in(Volts), voltage.in(Volts)),
						null, this));
		poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
				getRotation2d(), getLeftPositionMeters(), getRightPositionMeters(),
				getPose());
		registerSelfCheckHardware();
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
				getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec()));
	}
	/** SIM ONLY */
	public void updateSim(double dtSeconds){
		io.updateSim(dtSeconds);
	}
	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
				.toWheelSpeeds(speeds);
		driveVelocity(wheelSpeeds);
	}

	private DifferentialDriveWheelPositions getWheelPositions() {
		return new DifferentialDriveWheelPositions(getLeftPositionMeters(),
				getRightPositionMeters());
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive", inputs);
		// Update odometry
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		if (debounce == 1 && isConnected()) {
			poseEstimator.resetPosition(inputs.gyroYaw,
					wheelPositions.getPositions(), getPose());
			debounce = 0;
		}
		ChassisSpeeds m_ChassisSpeeds = getChassisSpeeds();
		if (inputs.gyroConnected) {
			// Use the real gyro angle
			rawGyroRotation = inputs.gyroYaw;
		} else {
			rawGyroRotation = rawGyroRotation.plus(
					new Rotation2d(m_ChassisSpeeds.omegaRadiansPerSecond * .02));
		}
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
		pose = poseEstimator.updateWithTime(wheelPositions.getTimestamp(),
				getRotation2d(), wheelPositions.getPositions());
		collisionDetected = collisionDetected();
		DrivetrainS.super.periodic();
	}

	/** Run open loop at the specified voltage. */
	public void driveVolts(double leftVolts, double rightVolts) {
		io.setVoltage(leftVolts, rightVolts);
	}

	/** Run closed loop at the specified voltage. */
	public void driveVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
		double leftRadPerSec = wheelSpeeds.leftMetersPerSecond / WHEEL_RADIUS;
		double rightRadPerSec = wheelSpeeds.rightMetersPerSecond / WHEEL_RADIUS;
		io.setVelocity(leftRadPerSec, rightRadPerSec,
				feedforward.calculate(leftRadPerSec),
				feedforward.calculate(rightRadPerSec));
	}

	/** Stops the drive. */
	@Override
	public void stopModules() { driveVelocity(new DifferentialDriveWheelSpeeds()); }

	/**
	 * Returns a command to run a quasistatic test in the specified direction.
	 */
	@Override
	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	@Override
	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}

	/** Returns the current odometry pose in meters. */
	@AutoLogOutput(key = "Odometry/Robot")
	@Override
	public Pose2d getPose() { return pose; }

	/** Resets the current odometry pose. */
	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(inputs.gyroYaw, getLeftPositionMeters(),
				getRightPositionMeters(), pose);
	}

	/** Returns the position of the left wheels in meters. */
	@AutoLogOutput
	public double getLeftPositionMeters() {
		return inputs.leftPositionRad * WHEEL_RADIUS;
	}

	/** Returns the position of the right wheels in meters. */
	@AutoLogOutput
	public double getRightPositionMeters() {
		return inputs.rightPositionRad * WHEEL_RADIUS;
	}

	/** Returns the velocity of the left wheels in meters/second. */
	@AutoLogOutput
	public double getLeftVelocityMetersPerSec() {
		return inputs.leftVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the velocity of the right wheels in meters/second. */
	@AutoLogOutput
	public double getRightVelocityMetersPerSec() {
		return inputs.rightVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the average velocity in radians/second. */
	public double getCharacterizationVelocity() {
		return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec)
				/ 2.0;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(io.getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = io.getSelfCheckingHardware();
		for (SelfChecking motor : driveHardware) {
			if (motor.getHardware() instanceof TalonFX) {
				orchestra.add((TalonFX) motor.getHardware());
			}
		}
		return orchestra;
	}

	@Override
	public double getCurrent() {
		if (inputs.leftCurrentAmps.length == 1) {
			return Math.abs(inputs.leftCurrentAmps[0])
					+ Math.abs(inputs.rightCurrentAmps[0]);
		}
		return Math.abs(inputs.leftCurrentAmps[0])
				+ Math.abs(inputs.leftCurrentAmps[1])
				+ Math.abs(inputs.rightCurrentAmps[0])
				+ Math.abs(inputs.rightCurrentAmps[1]);
	}

	@Override
	public SystemStatus getTrueSystemStatus() { return getSystemStatus(); }

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(1, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vxMetersPerSecond > 1.2
							|| getChassisSpeeds().vxMetersPerSecond < .8) {
						addFault(
								"[System Check] Forward speed did not reah target speed in time.",
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Rotation2d getRotation2d() { return rawGyroRotation; }

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }

	/**
	 * UNTESTED!
	 */
	@Override
	public void zeroHeading() {
		io.reset();
		debounce = 1;
	}

	@Override
	public boolean isConnected() { return inputs.gyroConnected; }

	private boolean collisionDetected() { return inputs.collisionDetected; }

	@Override
	public boolean isCollisionDetected() { return collisionDetected; }

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp", inputs.frontLeftDriveTemp);
		tempMap.put("FRDriveTemp", inputs.frontRightDriveTemp);
		tempMap.put("BLDriveTemp", inputs.backLeftDriveTemp);
		tempMap.put("BRDriveTemp", inputs.backRightDriveTemp);
		return tempMap;
	}

	@Override
	public void setDriveCurrentLimit(int amps) { io.setCurrentLimit(amps); }

	@Override
	public void setCurrentLimit(int amps) { setDriveCurrentLimit(amps); }
}
