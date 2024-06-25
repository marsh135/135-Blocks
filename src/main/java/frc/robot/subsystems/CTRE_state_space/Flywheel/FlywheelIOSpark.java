package frc.robot.subsystems.CTRE_state_space.Flywheel;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CTRE_state_space.StateSpaceConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingSparkBase;

public class FlywheelIOSpark implements FlywheelIO {
	private double appliedVolts = 0.0;
	private boolean closedLoop = true;
	private CANSparkBase flywheel;
	private RelativeEncoder encoder;
	/**
	 * This Plant holds a state-space model of our flywheel. It has the following
	 * properties: States: Velocity, in Rad/s. (will match Output) Inputs: Volts.
	 * Outputs: Velcoity, in Rad/s Flywheels require either an MOI or Kv and Ka,
	 * which are found using SysId.
	 */
	private final static LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId
			//.createFlywheelSystem(DCMotor.getNEO(1),StateSpaceConstants.Flywheel.MOI,StateSpaceConstants.Flywheel.flywheelGearing);
			.identifyVelocitySystem(StateSpaceConstants.Flywheel.flywheelValueHolder.getKv(), StateSpaceConstants.Flywheel.flywheelValueHolder.getKa());
	/**
	 * Kalman filters are optional, and help to predict the actual state of the
	 * system given a noisy encoder (which all encoders are!) Takes two empty
	 * N1's, our plant, and then our model accuracy in St. Devs and our Encoder
	 * value in St. Devs. Higher value means less trust, as an example 1 means we
	 * trust it 68% of all time.
	 */
	private final static KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N1(), Nat.N1(), flywheelPlant,
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanModel),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanEncoder), .02);
	/**
	 * A Linear Quadratic Regulator (LQR) controls linear systems. It minimizes
	 * cost, or voltage, to minimize state error. So, get to a position as fast,
	 * and efficient, as possible Our Qelms, or Q, is our PENALTY for deviation.
	 * Qelms is for our output, here being RPM, and R (Volts) is our penalty for
	 * control effort. Basically, high either of these means that it will take
	 * that more into account. A Qelms value of 100 means it REALLY cares about
	 * our RPM, and penalizes HEAVILY for incorrectness.
	 */
	private final static LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			flywheelPlant, VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRQelms),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRRVolts), .02);
	/**
	 * A state-space loop combines a controller, observer, feedforward, and plant
	 * all into one. It does everything! The random 12 is the upper limit on
	 * Voltage the loop can output. Never require more than 12 V, because.. we
	 * can't give it!
	 */
	private final static LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
			flywheelPlant, m_controller, m_observer, 12, .02);
	public FlywheelIOSpark(){
		if (StateSpaceConstants.Flywheel.motorVendor == MotorVendor.NEO_SPARK_MAX){
			flywheel = new CANSparkMax(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
		}else{
			flywheel = new CANSparkFlex(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
		}
		flywheel.enableVoltageCompensation(12);
		flywheel.setIdleMode(StateSpaceConstants.Flywheel.isBrake ? IdleMode.kBrake : IdleMode.kCoast);
		flywheel.setCANTimeout(250);
		flywheel.setInverted(StateSpaceConstants.Flywheel.inverted);
		flywheel.setSmartCurrentLimit(StateSpaceConstants.Flywheel.currentLimit);
		encoder = flywheel.getEncoder();
		flywheel.burnFlash();
		
		m_loop.setNextR(0); //go to zero
		m_loop.reset(VecBuilder.fill(0));
	}
	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		m_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() * StateSpaceConstants.Flywheel.flywheelGearing)));
		m_loop.predict(.02);
		if (closedLoop){
			appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
			flywheel.setVoltage(appliedVolts);
		}
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.currentAmps = new double[] {flywheel.getOutputCurrent()};
		inputs.flywheelTemp = flywheel.getMotorTemperature();
	}
	@Override
	/**Set the flywheel to a given velocity in radians per second. */
	public void setVelocity(double velocityRadPerSec){
		closedLoop = true;
		m_loop.setNextR(VecBuilder.fill(velocityRadPerSec));
	}
	@Override
	public void setVoltage(double volts){
		closedLoop = false;
		flywheel.setVoltage(volts);
	}
	@Override
	/**Stop the flywheel by telling it to go to 0 rpm. */
	public void stop(){
		m_loop.setNextR(VecBuilder.fill(0));
	}
	@Override
	/**Get the velocity error in radians per second */
	public double getError() {
		return Math
				.abs(encoder.getVelocity()*StateSpaceConstants.Flywheel.flywheelGearing  - m_loop.getNextR().get(0, 0));
	}
	@Override
	public List<SelfChecking> getSelfCheckingHardware(){
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Flywheel", flywheel));
		return hardware;
	}
}
