package frc.robot.subsystems.state_space.Elevator;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.selfCheck.drive.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorIOTalon implements ElevatorIO {
	private double appliedVolts = 0.0;
	private TalonFX elevator;
	private final StatusSignal<Double> elevatorPosition = elevator.getPosition();
	private final StatusSignal<Double> elevatorVelocity = elevator.getVelocity();
	private final StatusSignal<Double> elevatorAppliedVolts = elevator
			.getMotorVoltage();
	private final StatusSignal<Double> elevatorCurrent = elevator
			.getSupplyCurrent();
	private final StatusSignal<Double> elevatorTemp = elevator.getDeviceTemp();

	public ElevatorIOTalon() {
		elevator = new TalonFX(StateSpaceConstants.Elevator.kMotorID);
		var config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.Elevator.currentLimit;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.Elevator.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.Elevator.inverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		elevator.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, elevatorPosition,
				elevatorVelocity, elevatorAppliedVolts, elevatorCurrent,
				elevatorTemp);
		elevator.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity,
				elevatorAppliedVolts, elevatorCurrent, elevatorTemp);
		elevator.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.elevatorTemp = elevatorTemp.getValue();
		inputs.positionMeters = Units.rotationsToRadians(BaseStatusSignal
				.getLatencyCompensatedValue(elevatorPosition, elevatorVelocity, .2)
				* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.velocityMetersPerSec = Units
				.rotationsToRadians(elevatorVelocity.getValue()
						* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.currentAmps = new double[] {
				elevatorCurrent.getValue()
		};
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() { setVoltage(0); }

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("Elevator", elevator));
		return hardware;
	}
}