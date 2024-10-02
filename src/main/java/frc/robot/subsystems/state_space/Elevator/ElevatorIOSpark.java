package frc.robot.subsystems.state_space.Elevator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorIOSpark implements ElevatorIO {
	private double appliedVolts = 0.0;
	private CANSparkBase elevator;
	private RelativeEncoder encoder;
	private static final Executor CurrentExecutor = Executors
			.newFixedThreadPool(1);

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
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		elevator.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionMeters = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.elevatorTemp = elevator.getMotorTemperature();
		inputs.velocityMetersPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.Elevator.elevatorGearing);;
		inputs.currentAmps = new double[] { elevator.getOutputCurrent()
		};
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	public void setCurrentLimit(int amps) {
		CurrentExecutor.execute(() -> {
			elevator.setSmartCurrentLimit(amps);
		});
	}

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() { setVoltage(0); }

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Elevator", elevator));
		return hardware;
	}
}