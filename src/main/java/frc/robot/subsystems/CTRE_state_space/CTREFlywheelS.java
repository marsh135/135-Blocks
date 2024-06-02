package frc.robot.subsystems.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.Constants;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTREFlywheelS extends SubsystemBase {
	//initialize motors
	private static TalonFX flywheel = new TalonFX(CTRESpaceConstants.Flywheel.kMotorID);
	//System ID Routine
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); // for going FROM ZERO PER SECOND, this is 1v per 1sec.
	Measure<Voltage> holdVoltage = Volts.of(4); //what voltage should I hold during Quas test?
	Measure<Time> timeout = Seconds.of(10); //how many total seconds should I run the test, unless interrupted?
	//update cycle time
	private static double dtSeconds = .02; //20 ms
	private final VoltageOut m_voltReq = new VoltageOut(0.0);

	/**
	 * We cannot have multiple sysIdRoutines run on the same robot-cycle. If we
	 * want to run this and another sysIdRoutine, you must first POWER CYCLE the
	 * robot.
	 */

private final SysIdRoutine sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         rampRate,        // Use default ramp rate (1 V/s)
         holdVoltage, // Reduce dynamic step voltage to 4 to prevent brownout
         timeout,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> flywheel.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );
	/**
	 * This Plant holds a state-space model of our flywheel. It has the following
	 * properties: States: Velocity, in Rad/s. (will match Output) Inputs: Volts.
	 * Outputs: Velcoity, in Rad/s Flywheels require either an MOI or Kv and Ka,
	 * which are found using SysId.
	 */
	private final static LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId
			.identifyVelocitySystem(
					CTRESpaceConstants.Flywheel.flywheelValueHolder.getKv(),
					CTRESpaceConstants.Flywheel.flywheelValueHolder.getKa());
	/**
	 * Kalman filters are optional, and help to predict the actual state of the
	 * system given a noisy encoder (which all encoders are!) Takes two empty
	 * N1's, our plant, and then our model accuracy in St. Devs and our Encoder
	 * value in St. Devs. Higher value means less trust, as an example 1 means we
	 * trust it 68% of all time.
	 */
	private final static KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N1(), Nat.N1(), flywheelPlant,
			VecBuilder.fill(CTRESpaceConstants.Flywheel.m_KalmanModel),
			VecBuilder.fill(CTRESpaceConstants.Flywheel.m_KalmanEncoder),
			dtSeconds);
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
			flywheelPlant,
			VecBuilder.fill(CTRESpaceConstants.Flywheel.m_LQRQelms),
			VecBuilder.fill(CTRESpaceConstants.Flywheel.m_LQRRVolts), dtSeconds);
	/**
	 * A state-space loop combines a controller, observer, feedforward, and plant
	 * all into one. It does everything! The random 12 is the upper limit on
	 * Voltage the loop can output. Never require more than 12 V, because.. we
	 * can't give it!
	 */
	private final static LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
			flywheelPlant, m_controller, m_observer, 12, dtSeconds);
	/**
	 * Now, we define how it reacts in simulation. A gearing greater than 1 is a
	 * reduction.
	 */
	private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1),1/CTRESpaceConstants.Flywheel.flywheelGearing,.001);
	private double nextVoltage;

	public CTREFlywheelS() {
		var talonFXConfigurator = flywheel.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.Flywheel.statorCurrentLimit;
		motorConfig.MotorOutput.Inverted = CTRESpaceConstants.Flywheel.inverted;
		motorConfig.MotorOutput.NeutralMode = CTRESpaceConstants.Flywheel.mode;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.Flywheel.flywheelGearing;
		talonFXConfigurator.apply(motorConfig);
	
		//Tell our controllers that the encoders have a 19.5 ms delay (sparks)
		m_controller.latencyCompensate(flywheelPlant, dtSeconds, .01); //may be .02
		m_loop.setNextR(0); //go to zero
		m_loop.reset(getEncoderRotations());
	}

	private static Vector<N1> getEncoderRotations() {
		return VecBuilder
				.fill(flywheel.getVelocity().getValueAsDouble()*CTRESpaceConstants.Flywheel.flywheelGearing*60);
	}
	
	@Override
	public void periodic() {
		m_loop.correct(getEncoderRotations());
		m_loop.predict(dtSeconds);
		nextVoltage = m_loop.getU(0); //get model's control output
		flywheel.setControl(m_voltReq.withOutput(nextVoltage).withEnableFOC(true));
		if (Constants.currentMode == Constants.Mode.SIM){
			var talonFXSim = flywheel.getSimState();
			double motorVoltage = talonFXSim.getMotorVoltage();
			motorSim.setInputVoltage(motorVoltage);
			motorSim.update(dtSeconds);
			talonFXSim.setRawRotorPosition(motorSim.getAngularPositionRotations());
			talonFXSim.setRotorVelocity(Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec()));
		}
		if (CTRESpaceConstants.debug) {
			SmartDashboard.putNumber("Flywheel Speed", getEncoderRotations().get(0));
			SmartDashboard.putNumber("FlywheelError", CTREFlywheelS.getSpeedError());
		}
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	public static double getSpeedError() {
		return Math.abs(getEncoderRotations().get(0) - m_loop.getNextR().get(0,0));
	}

	/*
	 * Set RPM desired.
	 * @param speed in RPM
	 */
	public void setRPM(double rpm) { m_loop.setNextR(VecBuilder.fill(rpm)); }

	//Sim Only
	public double getDrawnCurrentAmps() {
		return motorSim.getCurrentDrawAmps();
	}
}
