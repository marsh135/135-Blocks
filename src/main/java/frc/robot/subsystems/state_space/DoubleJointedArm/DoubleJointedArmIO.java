package frc.robot.subsystems.state_space.DoubleJointedArm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface DoubleJointedArmIO {
	@AutoLog
	public static class DoubleJointedArmIOInputs {
		public double positionArmRads = 0.0;
		public double velocityArmRadsPerSec = 0.0;
		public double positionElbowRads = 0.0;
		public double velocityElbowRadsPerSec = 0.0;
		public double armTemp = 0.0;
		public double elbowTemp = 0.0;
		public double appliedArmVolts = 0.0;
		public double appliedElbowVolts = 0.0;
		public double[] currentAmps = new double[] {};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(DoubleJointedArmIOInputs inputs) {}

	/** Dumb control the doubleJointedArm using volts, no State Space */
	public default void setVoltage(List<Double> volts) {}

	/** Stop in open loop. */
	public default void stop() {}

	/** Gets the current draw from the implementation type */
																				/**
																				 * Get a list of the
																				 * SelfChecking
																				 * interface for all
																				 * hardware in that
																				 * implementation
																				 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}

