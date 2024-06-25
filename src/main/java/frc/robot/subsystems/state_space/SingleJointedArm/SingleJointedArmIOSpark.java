package frc.robot.subsystems.state_space.SingleJointedArm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.state_space.StateSpaceConstants;

public class SingleJointedArmIOSpark implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private boolean closedLoop = true;
	private CANSparkBase arm;
	private RelativeEncoder encoder;
//using sysId
	/*
	 * All SingleJointedArm Statespace uses an N2 at the first position, because we care about velocity AND position of the SingleJointedarm.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	private final LinearSystem<N2,N1,N1> m_SingleJointedArmPlant = 
	   LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(StateSpaceConstants.SingleJointedArm.armLength, StateSpaceConstants.SingleJointedArm.armMass), StateSpaceConstants.SingleJointedArm.armGearing);
	//private final LinearSystem<N2, N1, N1> m_SingleJointedArmPlant = LinearSystemId
	//		.identifyPositionSystem(CTRESpaceConstants.SingleJointedArm.armValueHolder.getKv(),
	//				CTRESpaceConstants.SingleJointedArm.armValueHolder.getKa());
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_SingleJointedArmPlant,
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_KalmanModelPosition,
					StateSpaceConstants.SingleJointedArm.m_KalmanModelVelocity),
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_KalmanEncoder), .02);
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_SingleJointedArmPlant,
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_LQRQelmsPosition,
					StateSpaceConstants.SingleJointedArm.m_LQRQelmsVelocity),
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_LQRRVolts), .02);
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant for easy control.
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_SingleJointedArmPlant, m_controller, m_observer, 12.0, .02);
	private static double m_velocity, m_position;
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our SingleJointedarm,
	 * allowing smooth motion control for the SingleJointedarm. Created with constraints based
	 * on the motor's free speed, but this will vary for every system, try tuning
	 * these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(StateSpaceConstants.SingleJointedArm.maxSpeed, //placeholder
					StateSpaceConstants.SingleJointedArm.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	//set our starting position for the SingleJointedarm
	/*
	 * TrapezoidProfile States are basically just a position in rads with a velocity in Rad/s
	 * Here, we provide our starting position.
	 */
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.SingleJointedArm.startingPosition, 0);
public SingleJointedArmIOSpark(){
		if (StateSpaceConstants.SingleJointedArm.motorVendor == MotorVendor.NEO_SPARK_MAX){
			arm = new CANSparkMax(StateSpaceConstants.SingleJointedArm.kMotorID, MotorType.kBrushless);
		}else{
			arm = new CANSparkFlex(StateSpaceConstants.SingleJointedArm.kMotorID, MotorType.kBrushless);
		}
		arm.enableVoltageCompensation(12);
		arm.setIdleMode(StateSpaceConstants.SingleJointedArm.isBrake ? IdleMode.kBrake : IdleMode.kCoast);
		arm.setCANTimeout(250);
		arm.setInverted(StateSpaceConstants.SingleJointedArm.inverted);
		arm.setSmartCurrentLimit(StateSpaceConstants.SingleJointedArm.currentLimit);
		encoder = arm.getEncoder();
		arm.burnFlash();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
			m_velocity);
	}
	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		m_lastProfiledReference = m_profile.calculate(.02,
				m_lastProfiledReference, goal); //calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); //Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data
		m_loop.correct(VecBuilder.fill(m_position));
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(.02);
		// Send the new calculated voltage to the motors.
		appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
		if (closedLoop){
			arm.setVoltage(appliedVolts);
		}
		m_position = Units.rotationsToRadians(encoder.getPosition() * StateSpaceConstants.SingleJointedArm.armGearing);
		m_velocity =  Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() * StateSpaceConstants.SingleJointedArm.armGearing);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = m_position; 
		inputs.armTemp = arm.getMotorTemperature();
		inputs.velocityRadPerSec = m_velocity;
		inputs.currentAmps = new double[] {MathUtil.clamp(arm.getOutputCurrent(),-StateSpaceConstants.SingleJointedArm.currentLimit,StateSpaceConstants.SingleJointedArm.currentLimit)}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
	}
	@Override
	/**Set the arm to a given state. */
	public void setState(TrapezoidProfile.State state){
		goal = state;
		closedLoop = true;
	}
	@Override
	public void setVoltage(double volts){
		closedLoop = false;
		arm.setVoltage(volts);
	}
	@Override
	/**Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop(){
		goal = new TrapezoidProfile.State(m_position, 0);
	}
	@Override
	/**Get the velocity error in radians per second */
	public double getError() {
		return Math
				.abs(Units.rotationsToRadians(encoder.getPosition() * StateSpaceConstants.SingleJointedArm.armGearing) - m_loop.getNextR().get(0, 0));
	}
	@Override
	/**Get the setpoint */
	public double getSetpoint(){
		return goal.position;
	}
}