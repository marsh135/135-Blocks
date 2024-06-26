package frc.robot.subsystems.state_space.Flywheel;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface FlywheelIO {
	@AutoLog
	public static class FlywheelIOInputs{
		public double positionRad = 0.0;
		public double positionError = 0.0;
		public double velocityRadPerSec = 0.0;
		public double flywheelTemp = 0.0;
		public double appliedVolts = 0.0;
		public double[] currentAmps = new double[]{};
	}
	
  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}
  /** Dumb control the Flywheel using volts, no State Space*/
  public default void setVoltage(double volts){}
  /** Stop in open loop. */
  public default void stop() {}
  /** Gets the current draw from the implementation type */
  	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
