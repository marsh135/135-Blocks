package frc.robot.subsystems.state_space.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.state_space.StateSpaceConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ElevatorS extends SubsystemChecker {
	private final ElevatorIO io;
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
	private final SysIdRoutine sysId;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); // for going FROM ZERO PER SECOND, this is 1v per 1sec.
	Measure<Voltage> holdVoltage = Volts.of(4); //what voltage should I hold during Quas test?
	Measure<Time> timeout = Seconds.of(10); //how many total seconds should I run the test, unless interrupted?
	private static double m_velocity, m_position;
	//using sysId
	/*
	 * All Elevator Statespace uses an N2 at the first position, because we care about velocity AND position of the Elevator.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	public static final LinearSystem<N2, N1, N1> m_elevatorPlant = LinearSystemId
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
	private TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Elevator.startingPosition, 0);
	// Create a Mechanism2d display of an Elevator with a fixed ElevatorTower and moving Elevator.
	/*
	 * Mechanism2d is really just an output of the robot, used to debug Elevator movement in simulation.
	 * the Mech2d itself is the "canvas" the mechanisms (like Elevators) are put on. It will always be your chassis.
	 * A Root2d is the point at which the mechanism rotates, or starts at. Elevators are different, but here we
	 * simply get it's X and Y according to the robot, then make that the root.
	 * Finally, we make the Ligament itself, and append this to the root point, basically putting an object at
	 * an origin point RELATIVE to the robot
	 * It takes the current position of the Elevator, and is the only thing updated constantly because of that
	 */
	private final Mechanism2d m_mech2d = new Mechanism2d(
			StateSpaceConstants.Elevator.maxPosition + .25,
			StateSpaceConstants.Elevator.maxPosition + .25);
	private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot(
			"Elevator Root", StateSpaceConstants.Elevator.physicalX,
			StateSpaceConstants.Elevator.physicalY);
	private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
			new MechanismLigament2d("Elevator", inputs.positionMeters, 90));

	public ElevatorS(ElevatorIO io) {
		this.io = io;
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
						(state) -> Logger.recordOutput("ElevatorS/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)),
						null, this));
		registerSelfCheckHardware();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
				m_velocity);
	}

	@Override
	public void periodic() {
		m_position = inputs.positionMeters;
		m_velocity = inputs.velocityMetersPerSec;
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
		double appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
		io.setVoltage(appliedVolts);
		io.updateInputs(inputs);
		Logger.processInputs("ElevatorS", inputs);
		m_elevatorMech2d.setLength(m_loop.getXHat(0));
		//Push the mechanism to AdvantageScope
		Logger.recordOutput("ElevatorMechanism", m_mech2d);
		//calcualate arm pose
		var elevatorPose = new Pose3d(StateSpaceConstants.Elevator.simX,
				StateSpaceConstants.Elevator.simY,
				StateSpaceConstants.Elevator.simZ, new Rotation3d(0, 0, 0.0));
		Logger.recordOutput("Mechanism3d/Elevator/", elevatorPose);
	}

	/** Run open loop at the specified voltage. */
	public void runVolts(double volts) {
		io.setVoltage(volts);
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in ElevatorS, called everywhere
	 * else.
	 * 
	 * @param position IN meters
	 * @return state, pass through to setState
	 */
	public TrapezoidProfile.State createState(double position) {
		return new TrapezoidProfile.State(position, 0);
	}

	//Also overload the function to accept both angle in METERS and METER/s
	/**
	 * @param position in METERS
	 * @param velocity in METES/SECOND
	 * @return state, pass through to deployArm.
	 */
	public TrapezoidProfile.State createState(double position, double velocity) {
		return new TrapezoidProfile.State(position, velocity);
	}

	/**
	 * Checks if close to state, with less than specified degrees
	 * 
	 * @param degrees error in degrees
	 * @return true if at state
	 */
	public boolean isAtState(double degrees) {
		if (Math.abs(getError()) < Units.degreesToRadians(degrees)) {
			return true;
		}
		return false;
	}

	/** Run closed loop to the specified state. */
	public void setState(TrapezoidProfile.State state) {
		goal = limitState(state);
		// Log arm setpoint
		Logger.recordOutput("ElevatorS/SetStatePosition", state.position);
		Logger.recordOutput("ElevatorS/SetStateVelocity", state.velocity);
	}

	/**
	 * @return error in meters from RAW encoder
	 */
	public double getError() {
		return Math.abs(m_position - m_loop.getNextR().get(0, 0));
	}

	/** Stops the arm. */
	public void stop() { io.stop(); }

	public double getDistance() { return m_loop.getXHat(0); }

	public double getSetpoint() { return goal.position; }

	public double getVelocity() { return inputs.velocityMetersPerSec; }

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction).onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction).onlyWhile(withinLimits(direction));
	}

	public TrapezoidProfile.State limitState(TrapezoidProfile.State state) {
		if (state.position < startingState().position) {
			return startingState();
		} else if (state.position > maxState().position) {
			return maxState();
		}
		return state;
	}

	/**
	 * Given a direction, is the Elevator currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < StateSpaceConstants.Elevator.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//second set of conditionals (below) checks to see if the Elevator is within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > StateSpaceConstants.Elevator.maxPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//otherwise, we're in bounds!
		returnVal = () -> true;
		return returnVal;
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
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("ElevatorTemp", inputs.elevatorTemp);
		return tempMap;
	}

	@Override
	public void setCurrentLimit(int amps) { io.setCurrentLimit(amps); }

	@Override
	public double getCurrent() { return inputs.currentAmps[0]; }

	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> setState(createState(Units.feetToMeters(2))))
						.withTimeout(2.0), runOnce(() -> {
							if (getError() > Units.inchesToMeters(4)) {
								addFault(
										"[System Check] Elevator position off more than 4 inches. Set 2ft, got "
												+ Units.metersToFeet(getDistance()),
										false, true);
							}
						}), run(() -> setState(createState(Units.feetToMeters(0))))
								.withTimeout(2.0),
						runOnce(() -> {
							if (getError() > Units.inchesToMeters(4)) {
								addFault(
										"[System Check] Elevator position off more than 4 inches. Set 0ft, got "
												+ Units.metersToFeet(getDistance()),
										false, true);
							}
						}))
				.until(() -> !getFaults().isEmpty());
	}
}
