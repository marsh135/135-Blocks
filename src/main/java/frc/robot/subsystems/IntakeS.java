package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.autoCommands.AutonIntake;
import frc.robot.utils.SimShootNote;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class IntakeS extends SubsystemBase {
	//declarations of motors/encoders and limit switch
	public CANSparkMax primaryIntake = new CANSparkMax(
			Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
	public CANSparkMax deployIntake = new CANSparkMax(
			Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
	public static RelativeEncoder deployIntakeEncoder;
	public RelativeEncoder primaryIntakeEncoder;
	public static DutyCycleEncoder absDeployIntakeEncoder;
	public static ColorSensorV3 colorSensorV3 = new ColorSensorV3(
			Constants.IntakeConstants.colorSensorPort);
	public static ColorMatch colorMatch = new ColorMatch();
	public static Color detected = Color.kBlack;
	public static ColorMatchResult colorMatchResult;
	public static Thread sensorThread;
	public static int timesRan;
	public static double kP, kI, kD;
	public static PIDController autoIntakeController; //sadly cannot be system Id'd
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(3);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine armIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				setIntakeMotorVolts(volts.in(Volts));
			}, null, this));
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(DCMotor.getNEO(1).freeSpeedRadPerSec, //placeholder
					DCMotor.getNEO(1).freeSpeedRadPerSec / 2)); // Max acceleration (def accurate frfr)😈😈😈
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	/* 
	//using MOI
	private final LinearSystem<N2,N1,N1> m_armPlant = 
	    LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(Units.inchesToMeters(5), Units.lbsToKilograms(13)), 400);
	*/
	//using sysId
	private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId
			.identifyPositionSystem(
					Constants.IntakeConstants.StateSpace.kVVoltSecondsPerRotation,
					Constants.IntakeConstants.StateSpace.kAVoltSecondsSquaredPerRotation);
	// The observer fuses our encoder data and voltage inputs to reject noise.
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_armPlant, VecBuilder.fill(0.015, 0.17), // How accurate we
			// think our model is, in radians and radians/sec
			VecBuilder.fill(0.01), // How accurate we think our encoder position
			// data is. In this case we very highly trust our encoder position reading.
			0.020);
	// A LQR uses feedback to create voltage commands.
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_armPlant,
			VecBuilder.fill(Units.degreesToRadians(1.0),
					Units.degreesToRadians(10.0)), // qelms.
			// Position and velocity error tolerances, in radians and radians per second. Decrease
			// this
			// to more heavily penalize state excursion, or make the controller behave more
			// aggressively. In this example we weight position much more highly than velocity, but
			// this
			// can be tuned to balance the two.
			VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
			// heavily penalize control effort, or make the controller less aggressive. 12 is a good
			// starting point because that is the (approximate) maximum voltage of a battery.
			0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant for easy control.
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_armPlant, m_controller, m_observer, 12.0, 0.020);
	private double m_velocity = 0;
	private static double m_position = 0;
	private double m_oldPosition = 0;
	private TrapezoidProfile.State goal = new TrapezoidProfile.State(
			Constants.IntakeConstants.deployIntakeInnerBound, 0);
	//sim values
	private SingleJointedArmSim simArm = new SingleJointedArmSim(m_armPlant,
			DCMotor.getNEO(1), 400, Units.inchesToMeters(5),
			Constants.IntakeConstants.deployIntakeInnerBound,
			Constants.IntakeConstants.deployIntakeOuterBound, true,
			Constants.IntakeConstants.deployIntakeInnerBound);
	// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
	private final Mechanism2d m_mech2d = new Mechanism2d(
			Units.inchesToMeters(29.5), Units.inchesToMeters(29.5));
	private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot",
			Units.inchesToMeters(23), Units.inchesToMeters(29.5 / 2));
	private final MechanismLigament2d m_arm = m_armPivot
			.append(new MechanismLigament2d("Arm", Units.inchesToMeters(5),
					m_position, 1, new Color8Bit(Color.kYellow)));

	public IntakeS() {
		timesRan = 0;
		colorMatch.addColorMatch(Constants.IntakeConstants.noteColor);
		colorMatch.addColorMatch(Color.kBlue);
		colorMatch.addColorMatch(Color.kRed);
		colorMatch.addColorMatch(Color.kGray);
		colorMatch.addColorMatch(Color.kWhite);
		//sets intake motors to reversed, sets idleMode to brake
		primaryIntake
				.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
		deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);
		primaryIntake.setIdleMode(IdleMode.kBrake);
		deployIntake.setIdleMode(IdleMode.kBrake);
		//creates encoders and makes them work with the gear ratios
		primaryIntakeEncoder = primaryIntake.getEncoder();
		primaryIntakeEncoder.setPositionConversionFactor(
				Constants.IntakeConstants.primaryIntakeGearRatio);
		primaryIntakeEncoder.setVelocityConversionFactor(
				Constants.IntakeConstants.primaryIntakeGearRatio);
		deployIntakeEncoder = deployIntake.getEncoder();
		absDeployIntakeEncoder = new DutyCycleEncoder(
				Constants.IntakeConstants.intakeAbsEncoderID);
		//sets changes to motor (resource intensive, ONLY CALL ON INITIALIZATION)
		primaryIntake.burnFlash();
		deployIntake.burnFlash();
		kP = Constants.IntakeConstants.PIDConstants.P;
		kI = Constants.IntakeConstants.PIDConstants.I;
		kD = Constants.IntakeConstants.PIDConstants.D;
		SmartDashboard.putNumber("P Gain Auto Intake", kP);
		SmartDashboard.putNumber("I Gain Auto Intake", kI);
		SmartDashboard.putNumber("D Gain Auto Intake", kD);
		if (Robot.isSimulation()){
			SmartDashboard.putData("Mech2d", m_mech2d);
		}
		autoIntakeController = new PIDController(kP, kI, kD);
		//autoIntakeController.enableContinuousInput(-180, 180);
		//autoIntakeController.setIntegratorRange(-Constants.DriveConstants.kMaxTurningSpeedRadPerSec/2, Constants.DriveConstants.kMaxTurningSpeedRadPerSec/2);
		//Color sensor thread
		m_loop.reset(VecBuilder.fill(getDistance(), getVelocity()));
		m_lastProfiledReference = new TrapezoidProfile.State(getDistance(),
				getVelocity());
	}

	/**
	 * Returns a command that will
	 * execute a quasistatic test in the
	 * given direction.
	 *
	 * @param direction The direction
	 *                     (forward or
	 *                     reverse) to
	 *                     run the test
	 *                     in
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return armIdRoutine.quasistatic(direction)
				.onlyWhile(intakeWithinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return armIdRoutine.dynamic(direction)
				.onlyWhile(intakeWithinLimits(direction));
	}

	@Override
	public void periodic() {
		updateEncoders();
		//sets values to SmartDashboard periodically
		SmartDashboard.putNumber("Deploy Intake Abs", getDistance());
		SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
		double p = SmartDashboard.getNumber("P Gain Auto Intake",
				Constants.IntakeConstants.PIDConstants.P);
		double i = SmartDashboard.getNumber("I Gain Auto Intake",
				Constants.IntakeConstants.PIDConstants.I);
		double d = SmartDashboard.getNumber("D Gain Auto Intake",
				Constants.IntakeConstants.PIDConstants.D);
		if ((p != kP)) {
			autoIntakeController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			autoIntakeController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			autoIntakeController.setD(d);
			kD = d;
		}
		m_lastProfiledReference = m_profile.calculate(0.020,
				m_lastProfiledReference, goal);
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity);
		// Correct our Kalman filter's state vector estimate with encoder data.
		if (Robot.isReal()) {
			m_loop.correct(VecBuilder.fill(getDistance()));
		}
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(0.020);
		// Send the new calculated voltage to the motors.
		// voltage = duty cycle * battery voltage, so
		// duty cycle = voltage / battery voltage
		double nextVoltage = m_loop.getU(0);
		if (Robot.isReal()) {
			deployIntake.setVoltage(nextVoltage);
		} else {
			simArm.setInput(nextVoltage);
			simArm.update(.02);
			m_arm.setAngle(m_position);
		}
		Logger.recordOutput("MyMechanism", m_mech2d);
		//calcualate arm pose
		var armPose = new Pose3d(0.292, 0, 0.1225,
				new Rotation3d(0, -Units.degreesToRadians(
						m_position - Constants.IntakeConstants.intakeOffset + 8),
						0.0));
		Logger.recordOutput("Mechanism3d/", armPose);
		SmartDashboard.putNumber("Angle Error", getError());
	}

	public double getIntakeAngle() {
		return getDistance() - IntakeConstants.intakeOffset;
	}

	public static double getDistance() { return m_position; }

	public double getVelocity() { return m_velocity; }

	public boolean intakeWithinBounds() {
		if (Robot.isSimulation())
			return true;
		return getIntakeAngle() > CameraS.getDesiredShooterLowerBound()
				&& getIntakeAngle() < CameraS.getDesiredShooterUpperBound()
				|| (getIntakeAngle() > 42 && CameraS.getDesiredShooterAngle() > 42);
	}

	/**
	 * power can be in ANY type, percent
	 * or voltage.
	 * 
	 * @param power
	 * @return
	 */
	public BooleanSupplier intakeWithinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < IntakeConstants.deployIntakeInnerBound * .85) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//second set of conditionals (below) checks to see if the arm is within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > IntakeConstants.deployIntakeOuterBound * .85) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		returnVal = () -> true;
		return returnVal;
	}

	public static boolean noteIsLoaded() {
		if (colorMatchResult.color == IntakeConstants.noteColor) {
			return true;
		} else {
			return false;
		}
	}

	public static int cloestNoteIndex;

	public Translation2d getClosestNote() {
		//Obnoxiously high distance to be overrode
		Translation2d closestTrans = new Translation2d();
		double closestPoseDistance = 9999; //hopefully something is closer than 9999 meters
		for (int i = 0; i < SimShootNote.getState().length; i++) {
			if (SimShootNote.getState()[i].getTranslation().toTranslation2d().getDistance(
					SwerveS.getPose().getTranslation()) < closestPoseDistance
					&& SimShootNote.getState()[i].getZ() < Units.inchesToMeters(1.1)) 
			{
				closestTrans = SimShootNote.getState()[i].getTranslation().toTranslation2d();
				cloestNoteIndex = i;
				closestPoseDistance = SimShootNote.getState()[i].getTranslation().toTranslation2d().getDistance(
					SwerveS.getPose().getTranslation());
			}
		}
		if (closestPoseDistance == 9999) {
			SimShootNote.resetNotes();
			return getClosestNote();
		}
		return closestTrans;
	}

	public TrapezoidProfile.State insideBotState() {
		return new TrapezoidProfile.State(
				Constants.IntakeConstants.deployIntakeInnerBound, 0);
	}

	public TrapezoidProfile.State outsideBotState() {
		return new TrapezoidProfile.State(
				Constants.IntakeConstants.deployIntakeOuterBound, 0);
	}

	/**
	 * For cleanliness of code, create
	 * State is in IntakeS, called
	 * everywhere else.
	 * 
	 * @param angle IN DEGREES
	 * @return state, pass through to
	 *         deployIntake.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(angle, 0);
	}

	/**
	 * Checks if close to state, with
	 * less than 2 degrees being "good"
	 * 
	 * @return true if at state
	 */
	public boolean isAtState() {
		if (Math.abs(getError()) < Units.degreesToRadians(2)) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if close to state, with
	 * less than 2 degrees being "good"
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
	 * @return error in degrees?
	 */
	public double getError() {
		return m_position - goal.position; //in radians
	}

	public void deployIntake(TrapezoidProfile.State state) { goal = state; }

	public void setPrimaryIntake(double power) {
		// sets the primary intake, comment below is a deadband check
		//power = power <= 0.1 ? 0.1 : power;
		primaryIntake.set(power);
	}

	public void updateEncoders() {
		if (Robot.isReal()) {
			m_position = absDeployIntakeEncoder.getAbsolutePosition()
					* Constants.IntakeConstants.absIntakeEncoderConversionFactor
					- Constants.IntakeConstants.absIntakeEncoderOffset;
			m_velocity = m_position - m_oldPosition; //since called every 20 ms
			m_oldPosition = m_position;
		} else {
			m_position = simArm.getAngleRads();
			m_velocity = simArm.getVelocityRadPerSec();
		}
	}

	public void setIntakeMotorVolts(double volts) {
		deployIntake.setVoltage(volts);
	}

	public void pullBackNote() {
		new Thread(() -> {
			Timer timer = new Timer();
			timer.start();
			while (timer.get() < .15) {
				setPrimaryIntake(-0.5);
			}
			timer.reset();
			while (timer.get() < .15) {
				setPrimaryIntake(0.2);
			}
			setPrimaryIntake(0);
			AutonIntake.allClear = true;
		}).start();
	}
}