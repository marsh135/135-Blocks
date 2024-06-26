package frc.robot.subsystems.state_space.Elevator;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingSparkBase;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorIOSpark implements ElevatorIO {
	private double appliedVolts = 0.0;
	private boolean closedLoop = true;
	private static double m_velocity, m_position;
	private CANSparkBase elevator;
	private RelativeEncoder encoder;
	//using sysId
	/*
	 * All Elevator Statespace uses an N2 at the first position, because we care about velocity AND position of the Elevator.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	private final LinearSystem<N2, N1, N1> m_elevatorPlant = LinearSystemId
			.createElevatorSystem(DCMotor.getNEO(1),
					StateSpaceConstants.Elevator.carriageMass,
					StateSpaceConstants.Elevator.drumRadius,
					StateSpaceConstants.Elevator.elevatorGearing);
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_elevatorPlant,
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanModelPosition,
					StateSpaceConstants.Elevator.m_KalmanModelVelocity), // How accurate we
			// think our model is, in meters and meters/second.
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanEncoder), // How accurate we think our encoder position
			// data is. In this case we very highly trust our encoder position reading.
			.02);
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_elevatorPlant,
			VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
			// and velocity error tolerances, in meters and meters per second. We care about pos more than velocity.
			VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
			// heavily penalize control effort
			.02);
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_elevatorPlant, m_controller, m_observer, 12.0, .02);
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our
	 * Elevator, allowing smooth motion control for the Elevator. Created with
	 * constraints based on the motor's free speed, but this will vary for every
	 * system, try tuning these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(StateSpaceConstants.Elevator.maxSpeed, //placeholder
					StateSpaceConstants.Elevator.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	//set our starting position for the Elevator
	/*
	* TrapezoidProfile States are basically just a position in rads with a velocity in Rad/s
	* Here, we provide our starting position.
	*/
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Elevator.startingPosition, 0);

	public ElevatorIOSpark() {
		if (StateSpaceConstants.Elevator.motorVendor == MotorVendor.NEO_SPARK_MAX) {
			elevator = new CANSparkMax(StateSpaceConstants.Elevator.kMotorID,
					MotorType.kBrushless);
		} else {
			elevator = new CANSparkFlex(StateSpaceConstants.Elevator.kMotorID,
					MotorType.kBrushless);
		}
		elevator.enableVoltageCompensation(12);
		elevator
				.setIdleMode(StateSpaceConstants.Elevator.isBrake ? IdleMode.kBrake
						: IdleMode.kCoast);
		elevator.setCANTimeout(250);
		elevator.setInverted(StateSpaceConstants.Elevator.inverted);
		elevator.setSmartCurrentLimit(StateSpaceConstants.Elevator.currentLimit);
		encoder = elevator.getEncoder();
		elevator.burnFlash();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
				m_velocity);
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
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
		if (closedLoop) {
			elevator.setVoltage(appliedVolts);
		}
		m_position = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.Elevator.elevatorGearing);
		m_velocity = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.appliedVolts = appliedVolts;
		inputs.positionMeters = m_position;
		inputs.setpointMeters = goal.position;
		inputs.errorMeters = Math.abs(Units
		.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.Elevator.elevatorGearing)
		- m_loop.getNextR().get(0, 0));
		inputs.elevatorTemp = elevator.getMotorTemperature();
		inputs.velocityMetersPerSec = m_velocity;
		inputs.currentAmps = new double[] {
				MathUtil.clamp(elevator.getOutputCurrent(),
						-StateSpaceConstants.Elevator.currentLimit,
						StateSpaceConstants.Elevator.currentLimit)
		}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
	}

	@Override
	/** Set the elevator to a given state. */
	public void setState(TrapezoidProfile.State state) {
		goal = state;
		closedLoop = true;
	}

	@Override
	public void setVoltage(double volts) {
		closedLoop = false;
		elevator.setVoltage(volts);
	}

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() { goal = new TrapezoidProfile.State(m_position, 0); }

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Elevator", elevator));
		return hardware;
	}
}