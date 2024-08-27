package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Mecanum.MecanumIOInputsAutoLogged;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
//import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.Sensors.GyroIOSim;
import frc.robot.utils.maths.SwerveStateProjection;

//import frc.robot.utils.maths.SwerveStateProjection;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

//import java.util.Arrays;
import java.util.function.Consumer;

/**
 * Simulates the dynamics of a mecanum robot. Uses essentially the same code as
 * a swerve drive sim (physics included). Takes motor behaviors from ModuleIOSim
 * and feeds the values back as simulated "encoder readings"
 */
public class MecanumDriveSimulation extends HolonomicChassisSimulation {
	private final MecanumIOSim mecanumIOSim;
	private final GyroIOSim gyroIOSim;
	private final static MecanumDriveKinematics kinematics;
	private final Consumer<Pose2d> resetOdometryCallBack;
	private final MecanumIOInputsAutoLogged mecanumIOInputsAutoLogged = new MecanumIOInputsAutoLogged();
	private double gForce = 0.0; //G
	public double convertRadPerSecondtoMeterPerSecond(double radPerSecond){
		return radPerSecond*TrainConstants.kDriveMotorGearRatio*TrainConstants.kWheelDiameter/2;
	}
	public MecanumDriveSimulation(RobotProfile robotProfile, GyroIOSim gyroIOSim,
			MecanumDriveKinematics kinematics, Pose2d startingPose,
			Consumer<Pose2d> resetOdometryCallBack) {
		super(robotProfile, startingPose);
		this.gyroIOSim = gyroIOSim;
		this.mecanumIOSim = new MecanumIOSim(gyroIOSim);
		this.kinematics = kinematics;
		this.resetOdometryCallBack = resetOdometryCallBack;
		resetOdometryToActualRobotPose();
		System.out.println("Mecanum drive sim profile: " + robotProfile);
	}

	public void resetOdometryToActualRobotPose() {
		resetOdometryCallBack.accept(getObjectOnFieldPose2d());
	}

	@Override
	public void updateSimulationSubPeriod(int iterationNum,
			double subPeriodSeconds) {
		mecanumIOSim.updateSim(subPeriodSeconds);
		double[] theoreticalWheelSpeeds = {convertRadPerSecondtoMeterPerSecond(mecanumIOInputsAutoLogged.leftFrontVelocityRadPerSec), convertRadPerSecondtoMeterPerSecond(mecanumIOInputsAutoLogged.rightFrontVelocityRadPerSec), convertRadPerSecondtoMeterPerSecond(mecanumIOInputsAutoLogged.leftBackVelocityRadPerSec), convertRadPerSecondtoMeterPerSecond(mecanumIOInputsAutoLogged.backRightDriveTemp)};
		final ChassisSpeeds mecanumTheoreticalSpeeds = kinematics
				.toChassisSpeeds(new MecanumDriveWheelSpeeds(theoreticalWheelSpeeds[0],theoreticalWheelSpeeds[1],theoreticalWheelSpeeds[2],theoreticalWheelSpeeds[3]));
		super.simulateChassisBehaviorWithRobotRelativeSpeeds(
				mecanumTheoreticalSpeeds);
		final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
		final MecanumDriveWheelSpeeds actualModuleFloorSpeeds = kinematics
				.toWheelSpeeds(instantVelocityRobotRelative);
		updateGyroSimulationResults(gyroIOSim,
				super.getObjectOnFieldPose2d().getRotation(),
				super.getAngularVelocity(), gForce, iterationNum);
		//updateMecanumSimulationResults(null, subPeriodSeconds, iterationNum, subPeriodSeconds, instantVelocityRobotRelative);
		

	}

	private final Vector2 previousDesiredLinearMotionPercent = new Vector2();

	@Override
	protected void simulateChassisTranslationalBehavior(
			Vector2 desiredLinearMotionPercent) {
		super.simulateChassisTranslationalBehavior(desiredLinearMotionPercent);
		final double dTheta = previousDesiredLinearMotionPercent
				.getAngleBetween(desiredLinearMotionPercent),
				desiredMotionDirectionChangingRateOmega = dTheta
						/ (Robot.defaultPeriodSecs
								/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD),
				centripetalForce = desiredMotionDirectionChangingRateOmega
						* getLinearVelocity().getMagnitude() * profile.robotMass;
		super.applyForce(Vector2.create(
				MathUtil.clamp(centripetalForce, -profile.frictionForce,
						profile.frictionForce),
				getLinearVelocity().getDirection() + Math.toRadians(90)));
		//calculate gForce
		gForce = Math
				.sqrt(Math.pow(getLinearVelocity().x, 2)
						+ Math.pow(getLinearVelocity().y, 2))
				/ DriveConstants.RobotPhysicsSimulationConfigs.FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ;
		previousDesiredLinearMotionPercent.set(desiredLinearMotionPercent);
	}

	private static void updateGyroSimulationResults(GyroIOSim gyroIOSim,
			Rotation2d currentFacing, double angularVelocityRadPerSec,
			double gForce, int iterationNum) {
		final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
		results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
		results.gForce = gForce;
		results.odometryYawPositions[iterationNum] = currentFacing;
		results.hasReading = true;
	}

	private static void updateMecanumSimulationResults(
			MecanumDriveWheelSpeeds speeds, double robotMaxVelocity,
			int simulationIteration, double periodSeconds, double[] theoreticalWheelSpeeds,
			ChassisSpeeds instantSpeed) {
					
			double[] degreeAngles = new double[4];
			double[] physicsAccurateWheelSpeeds = {speeds.frontLeftMetersPerSecond/robotMaxVelocity,speeds.frontRightMetersPerSecond/robotMaxVelocity,speeds.rearLeftMetersPerSecond/robotMaxVelocity,speeds.rearRightMetersPerSecond/robotMaxVelocity};
			//Convert mecanum array into wheel speeds (the loop should always iterate 4 times)
			for (int i = 0; i < theoreticalWheelSpeeds.length; i++){
				degreeAngles[i] = (TrainConstants.mecanumInitialAngleOffsetDegrees + i*90)%360;

			}
			
		final MecanumIOSim.MecanumDrivePhysicsSimResults results = MecanumIOSim.physicsSimulationResults;
		
		results.driveWheelFinalVelocityRevolutionsPerSec = getActualDriveMotorRotterSpeedRevPerSec(
				
				);
		results.odometrySteerPositions[simulationIteration] = moduleFreeSwerveSpeed.angle;
		if ((Math.sqrt(Math.pow(instantSpeed.vxMetersPerSecond, 2)
				+ Math.pow(instantSpeed.vyMetersPerSecond, 2)) > 0.1)
				|| (Math.sqrt(
						Math.pow(instantSpeed.omegaRadiansPerSecond, 2)) > 0.05)) {
			results.negateFF = false;
		} else {
			results.negateFF = true;
		}
		results.driveWheelFinalRevolutions += results.driveWheelFinalVelocityRevolutionsPerSec
				* periodSeconds;
		results.odometryDriveWheelRevolutions[simulationIteration] = results.driveWheelFinalRevolutions;
	}

	private static double getActualDriveMotorRotterSpeedRevPerSec(
			double moduleSpeedProjectedOnSwerveHeadingMPS,
			double moduleFreeSpeedMPS) {
			//Motor efficiency? I.E. how much of the free speed does it get at max speed? 
			final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = 0.8, 
 
				rotorSpeedMetersPerSecond;
		if (Math.abs(
				moduleFreeSpeedMPS - moduleSpeedProjectedOnSwerveHeadingMPS) < 2)
			rotorSpeedMetersPerSecond = moduleSpeedProjectedOnSwerveHeadingMPS;
		else
			rotorSpeedMetersPerSecond = moduleSpeedProjectedOnSwerveHeadingMPS
					* FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED
					+ moduleFreeSpeedMPS
							* (1 - FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED);
		final double rotorSpeedRadPerSec = rotorSpeedMetersPerSecond
				/ DriveConstants.TrainConstants.kWheelDiameter / 2;
		return Units.radiansToRotations(rotorSpeedRadPerSec);
		
	}

	public static final class OdometryThreadSim implements OdometryThread {
		@Override
		public void updateInputs(OdometryThreadInputs inputs) {
			inputs.measurementTimeStamps = new double[DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD];
			final double robotStartingTimeStamps = Logger.getTimestamp(),
					iterationPeriodSeconds = Robot.defaultPeriodSecs
							/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
			for (int i = 0; i < DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD; i++)
				inputs.measurementTimeStamps[i] = robotStartingTimeStamps
						+ i * iterationPeriodSeconds;
		}
	}
}
