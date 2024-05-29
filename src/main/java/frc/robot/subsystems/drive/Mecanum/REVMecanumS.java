package frc.robot.subsystems.drive.Mecanum;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.DrivetrainS;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.ejml.simple.UnsupportedOperation;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;

public class REVMecanumS implements DrivetrainS {
	private static CANSparkFlex
	frontLeftSparkFlex, 
	frontRightSparkFlex,
	backLeftSparkFlex,
	backRightSparkFlex;
	private static CANSparkMax 
	frontLeftSparkMax, 
	frontRightSparkMax, 
	backLeftSparkMax,
	backRightSparkMax;
	private static LinearSystem<N2, N1, N1> 
	frontLeftSystem,
	frontRightSystem,
	backLeftSystem,
	backRightSystem;
	private static RelativeEncoder
	frontLeftRelativeEncoder,
	frontRightRelativeEncoder,
	backLeftRelativeEncoder,
	backRightRelativeEncoder;
	private static AHRS gyro;
	private static MecanumDriveKinematics driveKinematics;
	private static MecanumDrivePoseEstimator drivePoseEstimator;


	public REVMecanumS(int frontLeftID, int frontRightID, int backLeftID,
			int backRightID,
			MotorConstantContainer frontLeftMotorConstantContainer,
			MotorConstantContainer frontRightMotorConstantContainer,
			MotorConstantContainer backLeftMotorConstantContainer,
			MotorConstantContainer backRightMotorConstantContainer) {
		switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				frontLeftSparkMax = new CANSparkMax(frontLeftID, MotorType.kBrushless);
				frontRightSparkMax = new CANSparkMax(frontRightID, MotorType.kBrushless);
				backLeftSparkMax = new CANSparkMax(backLeftID, MotorType.kBrushless);
				backRightSparkMax = new CANSparkMax(backRightID, MotorType.kBrushless);
				break;
			case VORTEX_SPARK_FLEX:
				frontLeftSparkFlex = new CANSparkFlex(frontLeftID, MotorType.kBrushless);
				frontRightSparkFlex = new CANSparkFlex(frontRightID, MotorType.kBrushless);
				backLeftSparkFlex = new CANSparkFlex(backLeftID, MotorType.kBrushless);
				backRightSparkFlex = new CANSparkFlex(backRightID, MotorType.kBrushless);
				break;
			default:
				throw new UnsupportedOperation("dawg you have to use a motortype");
				break;
		}
		frontLeftSystem = LinearSystemId.identifyPositionSystem(frontLeftMotorConstantContainer.getKv(), frontLeftMotorConstantContainer.getKa());
		frontRightSystem = LinearSystemId.identifyPositionSystem(frontRightMotorConstantContainer.getKv(), frontRightMotorConstantContainer.getKa());
		backLeftSystem = LinearSystemId.identifyPositionSystem(backLeftMotorConstantContainer.getKv(), backLeftMotorConstantContainer.getKa());
		backRightSystem = LinearSystemId.identifyPositionSystem(backRightMotorConstantContainer.getKv(), backRightMotorConstantContainer.getKa());
		gyro = new AHRS();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {}

	public ChassisSpeeds getChassisSpeeds() {}

	public void resetPose(Pose2d pose) {}

	void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {}

	Pose2d getPose() {}

	void stopModules() {}

	Rotation2d getRotation2d() {}

	Command sysIdDynamicTurn(Direction kreverse) {}

	Command sysIdQuasistaticTurn(Direction kforwards) {}

	Command sysIdDynamicDrive(Direction kforward) {}

	Command sysIdQuasistaticDrive(Direction kreverse) {}

	void zeroHeading() {}

	boolean isConnected() {}

	default void applyRequest() {
		throw new UnsupportedOperationException("No support for requests.");
	}
}
