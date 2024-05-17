package frc.robot.subsystems.state_space;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.state_space.StateSpaceConstants;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.BooleanSupplier;

// TODO: Comment all Sim and anything past Constructor
public class ArmS extends SubsystemBase {
	//initalize motors
	private CANSparkMax armMotor = new CANSparkMax(
			StateSpaceConstants.Arm.kMotorID, MotorType.kBrushless);
	private RelativeEncoder armEncoder;
	//update cycle time
	private static double dtSeconds = .02; //20 ms
	//live tuning, only enabled in Debug.
	//State Space
	//REFER TO FLYWHEEL FOR IN DEPTH EXPLANATION OF THE STATE-SPACE VARIABLES
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
	Measure<Voltage> holdVoltage = Volts.of(3);
	Measure<Time> timeout = Seconds.of(10);
	/**
	 * We cannot have multiple sysIdRoutines run on the same robot-cycle. If we
	 * want to run this and another sysIdRoutine, you must first POWER CYCLE the
	 * robot.
	 */
	SysIdRoutine armIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				setIntakeMotorVolts(volts.in(Volts));
			}, null, this));
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our arm,
	 * allowing smooth motion control for the arm. Created with constraints based
	 * on the motor's free speed, but this will vary for every system, try tuning
	 * these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(DCMotor.getNEO(1).freeSpeedRadPerSec, //placeholder
					DCMotor.getNEO(1).freeSpeedRadPerSec / 2));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	/* 
	//using MOI (currently broke estimate)
	private final LinearSystem<N2,N1,N1> m_armPlant = 
	    LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(Units.inchesToMeters(5), Units.lbsToKilograms(13)), 400);
	*/
	//using sysId
	/*
	 * All Arm Statespace uses an N2 at the first position, because we care about velocity AND position of the arm.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId
			.identifyPositionSystem(
					StateSpaceConstants.Arm.kArmVVoltSecondsPerRotation,
					StateSpaceConstants.Arm.kArmAVoltSecondsSquaredPerRotation);
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_armPlant,
			VecBuilder.fill(StateSpaceConstants.Arm.m_KalmanModelPosition,
					StateSpaceConstants.Arm.m_KalmanModelVelocity),
			VecBuilder.fill(StateSpaceConstants.Arm.m_KalmanEncoder), dtSeconds);
	// A LQR uses feedback to create voltage commands.
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_armPlant,
			VecBuilder.fill(StateSpaceConstants.Arm.m_LQRQelmsPosition,
					StateSpaceConstants.Arm.m_LQRQelmsVelocity),
			VecBuilder.fill(StateSpaceConstants.Arm.m_LQRRVolts), dtSeconds);
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant for easy control.
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_armPlant, m_controller, m_observer, 12.0, dtSeconds);
	private static double m_velocity, m_oldPosition, m_position;
	//set our starting position for the arm
	private TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Arm.startingPosition, 0);
	//Simulation (comment more)
	private SingleJointedArmSim simArm = new SingleJointedArmSim(m_armPlant,
			DCMotor.getNEO(1), StateSpaceConstants.Arm.armGearing,
			StateSpaceConstants.Arm.armLength,
			StateSpaceConstants.Arm.startingPosition,
			StateSpaceConstants.Arm.maxPosition, true,
			StateSpaceConstants.Arm.startingPosition);
	// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
	private final Mechanism2d m_mech2d = new Mechanism2d(
			DriveConstants.kChassisWidth, DriveConstants.kChassisLength);
	private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot",
			StateSpaceConstants.Arm.physicalX, StateSpaceConstants.Arm.physicalY);
	private final MechanismLigament2d m_arm = m_armPivot.append(
			new MechanismLigament2d("Arm", StateSpaceConstants.Arm.armLength,
					m_position, 1, new Color8Bit(Color.kYellow)));

	public ArmS() {
		//initalize arm motor
		armMotor.setInverted(StateSpaceConstants.Arm.inverted);
		armMotor.setIdleMode(StateSpaceConstants.Arm.mode);
		armEncoder = armMotor.getEncoder();
		armEncoder.setPositionConversionFactor(
				1 / StateSpaceConstants.Arm.armGearing);
		armEncoder.setVelocityConversionFactor(
				1 / StateSpaceConstants.Arm.armGearing);
		armMotor.burnFlash();
		m_loop.reset(VecBuilder.fill(getDistance(), getVelocity()));
		m_lastProfiledReference = new TrapezoidProfile.State(getDistance(),
				getVelocity());
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return armIdRoutine.quasistatic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return armIdRoutine.dynamic(direction).onlyWhile(withinLimits(direction));
	}

	public static double getDistance() { return m_position; }

	public double getVelocity() { return m_velocity; }

	public boolean withinBounds() {
		if (Robot.isSimulation())
			return true;
		return true;
	} //SETUP BOUNDS FOR ARM!

	/**
	 * Given a direction, is the arm currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < StateSpaceConstants.Arm.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//second set of conditionals (below) checks to see if the arm is within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > StateSpaceConstants.Arm.maxPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//otherwise, we're in bounds!
		returnVal = () -> true;
		return returnVal;
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Arm.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(StateSpaceConstants.Arm.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in IntakeS, called everywhere
	 * else.
	 * 
	 * @param angle IN DEGREES
	 * @return state, pass through to deployIntake.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);
	}

	/**
	 * Checks if close to state, with less than specified degrees
	 * 
	 * @param maximum error in degrees
	 * @return true if at state
	 */
	public boolean isAtState(double degrees) {
		if (Math.abs(getError()) < Units.degreesToRadians(degrees)) {
			return true;
		}
		return false;
	}

	/**
	 * @return error in radians
	 */
	public double getError() { return m_position - goal.position; }

	public void deployIntake(TrapezoidProfile.State state) { goal = state; }

	public void updateEncoders() {
		if (Robot.isReal()) {
			m_position = armEncoder.getPosition();
			m_velocity = m_position - m_oldPosition; //since called every 20 ms
			m_oldPosition = m_position;
		} else {
			m_position = simArm.getAngleRads();
			m_velocity = simArm.getVelocityRadPerSec();
		}
	}

	public void setIntakeMotorVolts(double volts) { armMotor.setVoltage(volts); }

	@Override
	public void periodic() {
		updateEncoders();
		if (StateSpaceConstants.debug) {
			//SmartDashboard Outputs
		}
		m_lastProfiledReference = m_profile.calculate(dtSeconds,
				m_lastProfiledReference, goal);
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity);
		// Correct our Kalman filter's state vector estimate with encoder data.
		if (Robot.isReal()) {
			m_loop.correct(VecBuilder.fill(getDistance()));
		}
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(dtSeconds);
		// Send the new calculated voltage to the motors.
		// voltage = duty cycle * battery voltage, so
		// duty cycle = voltage / battery voltage
		double nextVoltage = m_loop.getU(0);
		if (Robot.isReal()) {
			armMotor.setVoltage(nextVoltage);
		} else {
			simArm.setInput(nextVoltage);
			simArm.update(dtSeconds);
			m_arm.setAngle(m_position);
		}
		Logger.recordOutput("MyMechanism", m_mech2d);
		//calcualate arm pose
		var armPose = new Pose3d(0.292, 0, 0.1225,
				new Rotation3d(0, -Units.degreesToRadians(m_position), 0.0));
		Logger.recordOutput("Mechanism3d/", armPose);
		SmartDashboard.putNumber("Angle Error", getError());
	}
}
