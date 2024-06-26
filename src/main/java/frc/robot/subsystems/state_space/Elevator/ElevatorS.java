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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ElevatorS extends SubsystemChecker{
	private final ElevatorIO io;
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
	private final SysIdRoutine sysId;
	double setMeters = 0;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); // for going FROM ZERO PER SECOND, this is 1v per 1sec.
	Measure<Voltage> holdVoltage = Volts.of(4); //what voltage should I hold during Quas test?
	Measure<Time> timeout = Seconds.of(10); //how many total seconds should I run the test, unless interrupted?
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
private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot
	.append(new MechanismLigament2d("Elevator",
			inputs.positionMeters, 90));
	 public ElevatorS(ElevatorIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate,
                holdVoltage,
                timeout,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
	 registerSelfCheckHardware();
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
	 m_elevatorMech2d.setLength(inputs.positionMeters);
		//Push the mechanism to AdvantageScope
		Logger.recordOutput("ElevatorMechanism", m_mech2d);
		//calcualate arm pose
		var elevatorPose = new Pose3d(StateSpaceConstants.Elevator.simX,
				StateSpaceConstants.Elevator.simY, StateSpaceConstants.Elevator.simZ,
				new Rotation3d(0, 0, 0.0));
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
		return new TrapezoidProfile.State(StateSpaceConstants.Elevator.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in ElevatorS, called everywhere else.
	 * 
	 * @param angle IN RADS
	 * @return state, pass through to deployArm.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(angle, 0);
	}

	//Also overload the function to accept both angle in RADS and rad/s
	/**
	 * @param position in RADIANS
	 * @param velocity in RADIANS/SECOND
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
    io.setState(state);
	 setMeters = state.position;
    // Log arm setpoint
    Logger.recordOutput("Elevator/SetStatePosition", state.position);
	 Logger.recordOutput("Elevator/SetStateVelocity", state.velocity);

  }
  	/**
	 * @return error in meters
	 */
	public double getError() { return inputs.errorMeters; }
  /** Stops the arm. */
  public void stop() {
    io.stop();
  }
  public double getDistance() { return inputs.positionMeters; }

  public double getSetpoint() { return setMeters; }

  public double getVelocity() { return inputs.velocityMetersPerSec; }
	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction)
				.onlyWhile(withinLimits(direction));
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

	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("ElevatorTemp", inputs.elevatorTemp);
		return tempMap;
	}
	@Override
	public double getCurrent() {
		return inputs.currentAmps[0];
	 }
	 @Override
	 protected Command systemCheckCommand() {
		 return Commands.sequence(
				 run(() -> setState(createState(Units.feetToMeters(2))))
						 .withTimeout(2.0),
				 runOnce(() -> {
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
				 })).until(() -> !getFaults().isEmpty());
	 }
}
