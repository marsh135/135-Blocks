package frc.robot.subsystems.state_space.Flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelIOSim implements FlywheelIO {
	private double appliedVolts = 0.0;
	private boolean closedLoop = true;
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
	private FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant,DCMotor.getNEO(1),StateSpaceConstants.Flywheel.flywheelGearing,VecBuilder.fill(.5));
	public FlywheelIOSim(){
		m_loop.setNextR(0); //go to zero
		m_loop.reset(VecBuilder.fill(0));
	}
	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		m_loop.correct(VecBuilder.fill(flywheelSim.getAngularVelocityRadPerSec()));
		m_loop.predict(.02);
		if (closedLoop){
			appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
			flywheelSim.setInputVoltage(appliedVolts);
		}
		flywheelSim.update(.02);
		inputs.appliedVolts = appliedVolts;
		inputs.positionError = Math.abs(flywheelSim.getAngularVelocityRadPerSec() - m_loop.getNextR().get(0, 0));
		inputs.positionRad = 0.0; //no pos in flywheel
		inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
		inputs.currentAmps = new double[] {MathUtil.clamp(flywheelSim.getCurrentDrawAmps(),-StateSpaceConstants.Flywheel.currentLimit,StateSpaceConstants.Flywheel.currentLimit)}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
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
		flywheelSim.setInputVoltage(volts);
	}
	@Override
	/**Stop the flywheel by telling it to go to 0 rpm. */
	public void stop(){
		m_loop.setNextR(VecBuilder.fill(0));
	}
}
