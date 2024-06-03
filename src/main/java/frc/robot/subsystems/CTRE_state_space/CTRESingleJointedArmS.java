package frc.robot.subsystems.CTRE_state_space;



import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;
import frc.robot.utils.drive.DriveConstants;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.BooleanSupplier;

public class CTRESingleJointedArmS extends SubsystemBase {
	//initalize motors
	private TalonFX armMotor = new TalonFX(CTRESpaceConstants.SingleJointedArm.kMotorID);
	//update cycle time
	public static double dtSeconds = .02; //20 ms
	//live tuning, only enabled in Debug.
	/**
	 * State Space
	 * 
	 * @see FlywheelS FOR IN DEPTH EXPLANATION OF THE STATE-SPACE VARIABLE
	 */
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); // for going FROM ZERO PER SECOND, this is 1v per 1sec.
	Measure<Voltage> holdVoltage = Volts.of(4); //what voltage should I hold during Quas test?
	Measure<Time> timeout = Seconds.of(10); //how many total seconds should I run the test, unless interrupted?
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
         (volts) -> armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );
	//using sysId
	/*
	 * All SingleJointedArm Statespace uses an N2 at the first position, because we care about velocity AND position of the SingleJointedarm.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	//private final LinearSystem<N2,N1,N1> m_SingleJointedarmPlant = 
	//   LinearSystemId.createSingleJointedSingleJointedArmSystem(DCMotor.getKrakenX60Foc(1), SingleJointedSingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), Units.lbsToKilograms(6)), 150);
	private final LinearSystem<N2, N1, N1> m_SingleJointedArmPlant = LinearSystemId
			.identifyPositionSystem(CTRESpaceConstants.SingleJointedArm.armValueHolder.getKv(),
					CTRESpaceConstants.SingleJointedArm.armValueHolder.getKa());
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_SingleJointedArmPlant,
			VecBuilder.fill(CTRESpaceConstants.Elevator.m_KalmanModelPosition,
					CTRESpaceConstants.SingleJointedArm.m_KalmanModelVelocity),
			VecBuilder.fill(CTRESpaceConstants.SingleJointedArm.m_KalmanEncoder), dtSeconds);
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_SingleJointedArmPlant,
			VecBuilder.fill(CTRESpaceConstants.SingleJointedArm.m_LQRQelmsPosition,
					CTRESpaceConstants.SingleJointedArm.m_LQRQelmsVelocity),
			VecBuilder.fill(CTRESpaceConstants.SingleJointedArm.m_LQRRVolts), dtSeconds);
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant for easy control.
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_SingleJointedArmPlant, m_controller, m_observer, 12.0, dtSeconds);
	private static double m_velocity, m_oldPosition, m_position;
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our SingleJointedarm,
	 * allowing smooth motion control for the SingleJointedarm. Created with constraints based
	 * on the motor's free speed, but this will vary for every system, try tuning
	 * these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(CTRESpaceConstants.SingleJointedArm.maxSpeed, //placeholder
					CTRESpaceConstants.SingleJointedArm.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	//set our starting position for the SingleJointedarm
	/*
	 * TrapezoidProfile States are basically just a position in rads with a velocity in Rad/s
	 * Here, we provide our starting position.
	 */
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			CTRESpaceConstants.SingleJointedArm.startingPosition, 0);
	/**
	 * Constructs a single-jointed SingleJointedarm simulation with the given parameters.
	 * 
	 * @param SingleJointedarmGearing       The gearing ratio of the SingleJointedarm. >1 means reduction.
	 * @param SingleJointedarmLength        The length of the SingleJointedarm (in meters)
	 * @param minPosition      The minimum allowed position of the SingleJointedarm (in
	 *                            radians).
	 * @param maxPosition      The maximum allowed position of the SingleJointedarm (in
	 *                            radians).
	 * @param startingPosition The initial position of the SingleJointedarm (in radians).
	 *                            Calculates the position, velocity, and
	 *                            acceleration of the SingleJointedarm based on the applied
	 *                            motor voltage and time step. Provides current
	 *                            position, velocity, and voltage outputs for
	 *                            monitoring and control purposes.
	 */
	private SingleJointedArmSim simArm = new SingleJointedArmSim(m_SingleJointedArmPlant,
			DCMotor.getKrakenX60Foc(1), CTRESpaceConstants.SingleJointedArm.armGearing,
			CTRESpaceConstants.SingleJointedArm.armLength,
			CTRESpaceConstants.SingleJointedArm.startingPosition,
			CTRESpaceConstants.SingleJointedArm.maxPosition, false,
			CTRESpaceConstants.SingleJointedArm.startingPosition);
	// Create a Mechanism2d display of an SingleJointedArm with a fixed SingleJointedArmTower and moving SingleJointedArm.
	/*
	 * Mechanism2d is really just an output of the robot, used to debug SingleJointedarm movement in simulation.
	 * the Mech2d itself is the "canvas" the mechanisms (like SingleJointedarms) are put on. It will always be your chassis.
	 * A Root2d is the point at which the mechanism rotates, or starts at. Elevators are different, but here we
	 * simply get it's X and Y according to the robot, then make that the root.
	 * Finally, we make the Ligament itself, and append this to the root point, basically putting an object at
	 * an origin point RELATIVE to the robot
	 * It takes the current position of the SingleJointedarm, and is the only thing updated constantly because of that
	 */
	private final Mechanism2d m_mech2d = new Mechanism2d(
			DriveConstants.kChassisWidth, DriveConstants.kChassisLength);
	private final MechanismRoot2d m_SingleJointedarmPivot = m_mech2d.getRoot("SingleJointedArmPivot",
			CTRESpaceConstants.SingleJointedArm.physicalX, CTRESpaceConstants.SingleJointedArm.physicalY);
	private final MechanismLigament2d m_SingleJointedarm = m_SingleJointedarmPivot.append(
			new MechanismLigament2d("SingleJointedArm", CTRESpaceConstants.SingleJointedArm.armLength,
					Units.radiansToDegrees(m_lastProfiledReference.position), 1, new Color8Bit(Color.kYellow)));

	public CTRESingleJointedArmS() {
		//initalize SingleJointedarm motor
		var talonFXConfigurator = armMotor.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.SingleJointedArm.statorCurrentLimit;
		motorConfig.MotorOutput.Inverted = CTRESpaceConstants.SingleJointedArm.inverted;
		motorConfig.MotorOutput.NeutralMode = CTRESpaceConstants.SingleJointedArm.mode;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.SingleJointedArm.armGearing;
		talonFXConfigurator.apply(motorConfig);
	
		//reset our position (getDistance just because it's zero usually, and IF the robot died on field and rebooted, it'd still be accurate.)
		m_loop.reset(VecBuilder.fill(getDistance(), getVelocity()));
		m_lastProfiledReference = new TrapezoidProfile.State(getDistance(),
				getVelocity());
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction).onlyWhile(withinLimits(direction));
	}

	public static double getDistance() { return m_position; }

	public static double getSetpoint() { return goal.position; }

	public double getVelocity() { return m_velocity; }

	/**
	 * Given a direction, is the SingleJointedarm currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < CTRESpaceConstants.SingleJointedArm.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//second set of conditionals (below) checks to see if the SingleJointedarm is within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > CTRESpaceConstants.SingleJointedArm.maxPosition) {
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
				CTRESpaceConstants.SingleJointedArm.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(CTRESpaceConstants.SingleJointedArm.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in SingleJointedArmS, called everywhere else.
	 * 
	 * @param angle IN RADS
	 * @return state, pass through to deployIntake.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(angle, 0);
	}

	//Also overload the function to accept both angle in RADS and rad/s
	/**
	 * @param position in RADIANS
	 * @param velocity in RADIANS/SECOND
	 * @return state, pass through to deployIntake.
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

	/**
	 * @return error in radians
	 */
	public double getError() { return m_position - goal.position; }

	public void deployIntake(TrapezoidProfile.State state) { goal = state; }

	public void updateEncoders() {
		switch (Constants.currentMode) {
		case REAL:
			m_position = armMotor.getPosition().getValueAsDouble();
			m_velocity = m_position - m_oldPosition; //since called every 20 ms
			m_oldPosition = m_position;
			break;
		default:
			m_position = simArm.getAngleRads();
			m_velocity = simArm.getVelocityRadPerSec();
			m_SingleJointedarm.setAngle(m_position);
			break;
		}
	}

	public void setIntakeMotorVolts(double volts) { armMotor.setVoltage(volts); }

	@Override
	public void periodic() {
		updateEncoders();
		m_lastProfiledReference = m_profile.calculate(dtSeconds,
				m_lastProfiledReference, goal); //calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); //Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data ONLY if real
		if (Constants.currentMode == Mode.REAL) {
			m_loop.correct(VecBuilder.fill(getDistance()));
		}
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(dtSeconds);
		// Send the new calculated voltage to the motors.
		double nextVoltage = m_loop.getU(0);
		switch (Constants.currentMode) {
		case REAL:
			armMotor.setVoltage(nextVoltage);
			break;
		default:
			simArm.setInputVoltage(nextVoltage);
			simArm.update(dtSeconds);
			break;
		}
		//Push the mechanism to AdvantageScope
		Logger.recordOutput("SingleJointedArmMechanism", m_mech2d);
		//calcualate SingleJointedarm pose
		var SingleJointedarmPose = new Pose3d(CTRESpaceConstants.SingleJointedArm.simX, CTRESpaceConstants.SingleJointedArm.simY, CTRESpaceConstants.SingleJointedArm.simZ,
				new Rotation3d(0, -m_position, 0.0));
		Logger.recordOutput("Mechanism3d/SingleJointedArm/", SingleJointedarmPose);
		if (CTRESpaceConstants.debug) {
			SmartDashboard.putNumber("Angle Error SingleJointedARM", Units.radiansToDegrees(getError()));
			SmartDashboard.putNumber("SETPOINT SingleJointedARM",
					Units.radiansToDegrees(goal.position));
			SmartDashboard.putNumber("CURRENT WANTED SingleJointedARM",
					Units.radiansToDegrees(m_lastProfiledReference.position));
			SmartDashboard.putNumber("pos SingleJointedARM",
					Units.radiansToDegrees(m_position));
		}
	}
}