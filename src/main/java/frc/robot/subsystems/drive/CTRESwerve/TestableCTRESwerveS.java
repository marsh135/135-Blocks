package frc.robot.subsystems.drive.CTRESwerve;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SubsystemFault;
import java.util.List;
import java.util.ArrayList;

public class TestableCTRESwerveS extends SubsystemChecker
		implements DrivetrainS {
	final CTRESwerveS ctreSwerveS;
	TalonFX[][] motors;

	public TestableCTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			double OdometryUpdateFrequency, Telemetry logger,
			SwerveModuleConstants... modules) {
		this.ctreSwerveS = new CTRESwerveS(driveTrainConstants,
				OdometryUpdateFrequency, logger, modules);
		makeMotors();
	}

	/**
	 * Creates a CTRE Swerve Drivetrain. Does not have an updateOdometryFrequency
	 * since this is run in sim.
	 * 
	 * @param driveTrainConstants Example of this is in TunerConstants.java
	 * @param logger              //Where to log the data
	 * @param modules             //Which modules to use
	 */
	public TestableCTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			Telemetry logger, SwerveModuleConstants... modules) {
		this.ctreSwerveS = new CTRESwerveS(driveTrainConstants, logger, modules);
		makeMotors();
	}

	private TalonFX[][] makeMotors() {
		return new TalonFX[][] { { ctreSwerveS.getModule(0).getDriveMotor(),
				ctreSwerveS.getModule(0).getSteerMotor()
				}, { ctreSwerveS.getModule(1).getDriveMotor(),
						ctreSwerveS.getModule(1).getSteerMotor()
				}, { ctreSwerveS.getModule(2).getDriveMotor(),
						ctreSwerveS.getModule(2).getSteerMotor()
				}, { ctreSwerveS.getModule(3).getDriveMotor(),
						ctreSwerveS.getModule(3).getSteerMotor()
				}
		};
	}

	public void registerSelfCheckHardware() {
		super.registerHardware("IMU", ctreSwerveS.getPigeon2());
		super.registerHardware("FrontLeftDrive", motors[0][0]);
		super.registerHardware("FrontLeftTurn", motors[0][1]);
		super.registerHardware("FrontRightDrive", motors[1][0]);
		super.registerHardware("FrontRightTurn", motors[1][1]);
		super.registerHardware("BackLeftDrive", motors[2][0]);
		super.registerHardware("BackLeftTurn", motors[2][1]);
		super.registerHardware("BackRightDrive", motors[3][0]);
		super.registerHardware("BackRightTurn", motors[3][1]);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		for (var module : motors) {
			orchestra.add(module[0]);
			orchestra.add(module[1]);
		}
		return orchestra;
	}

	@Override
	public SystemStatus getSystemStatus() {
		SystemStatus worstStatus = SystemStatus.OK;
		for (SubsystemFault f : this.getFaults()) {
			if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
				if (f.isWarning) {
					if (worstStatus != SystemStatus.ERROR) {
						worstStatus = SystemStatus.WARNING;
					}
				} else {
					worstStatus = SystemStatus.ERROR;
				}
			}
		}
		return worstStatus;
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

	boolean frontLeftFinished = false;
	boolean frontRightFinished = false;
	boolean backLeftFinished = false;
	boolean backRightFinished = false;

	private Command swerveModuleTest(SwerveModule module, String moduleName) {
		return Commands.sequence(
				run(() -> module.apply(
						new SwerveModuleState(0,
								new Rotation2d(Units.degreesToRadians(90))),
						DriveRequestType.OpenLoopVoltage)).withTimeout(1.0),
				runOnce(() -> {
					if (module.getCurrentState().angle.getDegrees() < 70
							|| module.getCurrentState().angle.getDegrees() > 110) {
						addFault(
								"[System Check] Rotation Motor did not reach target position for "
										+ moduleName,
								false, true);
					}
				}), runOnce(() -> {
					module.getDriveMotor().setVoltage(1.2);
					module.getSteerMotor().stopMotor();
				}), Commands.waitSeconds(0.5), runOnce(() -> {
					if (module.getCurrentState().speedMetersPerSecond < 0.25) {
						addFault(
								"[System Check] Drive motor encoder velocity too slow for "
										+ moduleName,
								false, true);
					}
					module.getDriveMotor().stopMotor();
				}), Commands.waitSeconds(0.25),
				run(() -> module.apply(
						new SwerveModuleState(0,
								new Rotation2d(Units.degreesToRadians(0))),
						DriveRequestType.OpenLoopVoltage)).withTimeout(1.0),
				runOnce(() -> {
					if (Math.abs(module.getCurrentState().angle.getDegrees()) > 20) {
						addFault(
								"[System Check] Rotation did not reach target position for "
										+ moduleName,
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(runOnce(() -> {
					module.getDriveMotor().stopMotor();
					module.getSteerMotor().stopMotor();
					switch (moduleName) {
					case "FrontLeft":
						frontLeftFinished = true;
						break;
					case "FrontRight":
						frontRightFinished = true;
						break;
					case "BackLeft":
						backLeftFinished = true;
						break;
					default:
						backRightFinished = true;
						break;
					}
				}));
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				/*runOnce(
					() -> {
					  swerveModuleTest(ctreSwerveS.getModule(0),"FrontLeft").schedule();
					  swerveModuleTest(ctreSwerveS.getModule(1),"FrontRight").schedule();
					  swerveModuleTest(ctreSwerveS.getModule(2),"BackLeft").schedule();
					  swerveModuleTest(ctreSwerveS.getModule(3),"BackRight").schedule();
					}),
				// Hack to run module system checks since modules[] does not exist when this method is
				// called
				Commands.waitUntil(
					() ->
					frontLeftFinished && frontRightFinished && backLeftFinished && backRightFinished),*/
				run(() -> setChassisSpeeds(new ChassisSpeeds(1.5, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					for (int i = 0; i < ctreSwerveS
							.getState().ModuleStates.length; i++) {
						String name = DriveConstants.TrainConstants.ModulePosition
								.values()[i].name();
						SwerveModuleState module = ctreSwerveS
								.getState().ModuleStates[i];
						if (Math.abs(module.speedMetersPerSecond) < 1.3
								|| Math.abs(module.speedMetersPerSecond) > 1.7) {
							addFault(
									"[System Check] Drive motor encoder velocity too slow for "
											+ name + module.speedMetersPerSecond);
						}
						if (Math.abs(module.angle.getRadians()) > Units
								.degreesToRadians(10))
							if (Math.abs(module.angle.getDegrees() - 180) > 10) {
								addFault("[System Check] Turn angle off for " + name
										+ module.angle);
							}
					}
				}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1.5, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					for (int i = 0; i < ctreSwerveS
							.getState().ModuleStates.length; i++) {
						String name = DriveConstants.TrainConstants.ModulePosition
								.values()[i].name();
						SwerveModuleState module = ctreSwerveS
								.getState().ModuleStates[i];
						if (Math.abs(module.speedMetersPerSecond) < 1.3
								|| Math.abs(module.speedMetersPerSecond) > 1.7) {
							addFault(
									"[System Check] Drive motor encoder velocity too slow for "
											+ name + module.speedMetersPerSecond);
						}
						if (Math.abs(module.angle.getDegrees()) > 10)
							if (Math.abs(module.angle.getDegrees() - 180) > 10) {
								addFault("[System Check] Turn angle off for " + name
										+ module.angle);
							}
					}
				}),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
						.withTimeout(2.0))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		ctreSwerveS.setChassisSpeeds(speeds);
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return ctreSwerveS.getChassisSpeeds();
	}

	@Override
	public void resetPose(Pose2d pose) { ctreSwerveS.resetPose(pose); }

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		ctreSwerveS.newVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return ctreSwerveS.getPose(); }

	@Override
	public void stopModules() { ctreSwerveS.stopModules(); }

	@Override
	public Rotation2d getRotation2d() { return ctreSwerveS.getRotation2d(); }

	@Override
	public double getYawVelocity() { return ctreSwerveS.getYawVelocity(); }

	@Override
	public Twist2d getFieldVelocity() { return ctreSwerveS.getFieldVelocity(); }

	@Override
	public Command sysIdDynamicTurn(Direction direction) {
		return ctreSwerveS.sysIdDynamicTurn(direction);
	}

	@Override
	public Command sysIdQuasistaticTurn(Direction direction) {
		return ctreSwerveS.sysIdQuasistaticTurn(direction);
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return ctreSwerveS.sysIdDynamicDrive(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return ctreSwerveS.sysIdQuasistaticDrive(direction);
	}

	@Override
	public void zeroHeading() { ctreSwerveS.zeroHeading(); }

	@Override
	public boolean isConnected() { return ctreSwerveS.isConnected(); }

	@Override
	public void applyRequest() { ctreSwerveS.applyRequest(); }

	@Override
	public double getCurrent() {
		return 0;//ctreSwerveS.getCurrent(); //ENABLE IF WANT PROPER CURRENT, BE AWARE CAUSES SOME RESIM'S NEEDED!
	}
}
