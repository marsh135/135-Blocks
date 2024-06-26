package frc.robot.subsystems.state_space.SingleJointedArm;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingTalonFX;
import frc.robot.utils.state_space.StateSpaceConstants;

public class SingleJointedArmIOTalon implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private boolean closedLoop = true;
	private TalonFX arm;
	private final StatusSignal<Double> armPosition = arm.getPosition();
   private final StatusSignal<Double> armVelocity = arm.getVelocity();
   private final StatusSignal<Double> armAppliedVolts = arm.getMotorVoltage();
   private final StatusSignal<Double> armCurrent = arm.getSupplyCurrent();
	private final StatusSignal<Double> armTemp = arm.getDeviceTemp();
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
public SingleJointedArmIOTalon(){
		arm = new TalonFX(StateSpaceConstants.SingleJointedArm.kMotorID);
		var config = new TalonFXConfiguration();
    	config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.SingleJointedArm.currentLimit;
    	config.CurrentLimits.SupplyCurrentLimitEnable = true;
    	config.MotorOutput.NeutralMode = StateSpaceConstants.SingleJointedArm.isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.SingleJointedArm.inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    	arm.getConfigurator().apply(config);
 		BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armPosition, armVelocity, armAppliedVolts, armCurrent, armTemp);
      arm.optimizeBusUtilization();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
			m_velocity);
	}
	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		BaseStatusSignal.refreshAll(
			armPosition, armVelocity, armAppliedVolts, armCurrent, armTemp);
		
		m_lastProfiledReference = m_profile.calculate(.02,
				m_lastProfiledReference, goal); //calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); //Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data

		m_position = Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(armPosition, armVelocity, .2) * StateSpaceConstants.SingleJointedArm.armGearing);
		m_velocity =  Units.rotationsToRadians(armVelocity.getValue() * StateSpaceConstants.SingleJointedArm.armGearing);
		m_loop.correct(VecBuilder.fill(m_position));
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(.02);
		// Send the new calculated voltage to the motors.
		appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
		if (closedLoop){
			arm.setVoltage(appliedVolts);
		}
		inputs.appliedVolts = appliedVolts;
		inputs.armTemp = armTemp.getValue();
		inputs.errorRad = Math.abs(Units.rotationsToRadians(armPosition.getValue() * StateSpaceConstants.SingleJointedArm.armGearing) - m_loop.getNextR().get(0, 0));
		inputs.positionRad = m_position;
		inputs.setpointRad = goal.position; 
		inputs.velocityRadPerSec = m_velocity;
		inputs.currentAmps = new double[] {MathUtil.clamp(armCurrent.getValue(),-StateSpaceConstants.SingleJointedArm.currentLimit,StateSpaceConstants.SingleJointedArm.currentLimit)}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
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
	public List<SelfChecking> getSelfCheckingHardware(){
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("SingleArm", arm));
		return hardware;
	}
}