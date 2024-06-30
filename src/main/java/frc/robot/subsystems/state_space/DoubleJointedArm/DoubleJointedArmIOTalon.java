package frc.robot.subsystems.state_space.DoubleJointedArm;

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

public class DoubleJointedArmIOTalon implements DoubleJointedArmIO {
	private double armVolts = 0.0;
	private double elbowVolts = 0.0;
	private TalonFX arm;
	private TalonFX elbow;
	private final StatusSignal<Double> armPosition = arm.getPosition();
	private final StatusSignal<Double> armVelocity = arm.getVelocity();
	private final StatusSignal<Double> armAppliedVolts = arm.getMotorVoltage();
	private final StatusSignal<Double> armCurrent = arm.getSupplyCurrent();
	private final StatusSignal<Double> armTemp = arm.getDeviceTemp();
	private final StatusSignal<Double> elbowPosition = elbow.getPosition();
	private final StatusSignal<Double> elbowVelocity = elbow.getVelocity();
	private final StatusSignal<Double> elbowAppliedVolts = elbow
			.getMotorVoltage();
	private final StatusSignal<Double> elbowCurrent = elbow.getSupplyCurrent();
	private final StatusSignal<Double> elbowTemp = elbow.getDeviceTemp();

	public DoubleJointedArmIOTalon() {
		arm = new TalonFX(StateSpaceConstants.DoubleJointedArm.kArmMotorID);
		elbow = new TalonFX(StateSpaceConstants.DoubleJointedArm.kElbowMotorID);
		var config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.DoubleJointedArm.armCurrentLimit;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.DoubleJointedArm.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.DoubleJointedArm.armInverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		arm.getConfigurator().apply(config);
		config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.DoubleJointedArm.elbowCurrentLimit;
		config.MotorOutput.NeutralMode = StateSpaceConstants.DoubleJointedArm.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.DoubleJointedArm.elbowInverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		elbow.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, armPosition, armVelocity,
				armAppliedVolts, armCurrent, armTemp, elbowPosition, elbowVelocity,
				elbowAppliedVolts, elbowCurrent, elbowTemp);
		arm.optimizeBusUtilization();
		elbow.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(DoubleJointedArmIOInputs inputs) {
		BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts,
				armCurrent, armTemp, elbowPosition, elbowVelocity,
				elbowAppliedVolts, elbowCurrent, elbowTemp);
		arm.setVoltage(armVolts);
		inputs.appliedArmVolts = armAppliedVolts.getValue();
		inputs.positionArmRads = Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(armPosition, armVelocity) * StateSpaceConstants.DoubleJointedArm.armGearing);
		inputs.velocityArmRadsPerSec = Units.rotationsToRadians(armVelocity.getValue() * StateSpaceConstants.DoubleJointedArm.armGearing);
		inputs.armTemp = armTemp.getValue();
		elbow.setVoltage(elbowVolts);
		inputs.appliedElbowVolts = elbowAppliedVolts.getValue();
		inputs.positionElbowRads = Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(elbowPosition, elbowVelocity) * StateSpaceConstants.DoubleJointedArm.elbowGearing);
		inputs.velocityElbowRadsPerSec = Units.rotationsToRadians(elbowVelocity.getValue() * StateSpaceConstants.DoubleJointedArm.elbowGearing);
		inputs.elbowTemp = elbowTemp.getValue();
		inputs.currentAmps = new double[] {armCurrent.getValue(),elbowCurrent.getValue()};
	}

	@Override
	public void setVoltage(List<Double> volts) {
		armVolts = volts.get(0);
		elbowVolts = volts.get(1);
	}

	@Override
	/** Stop the arm by telling it to go to 0 arm. */
	public void stop() {
		armVolts = 0;
		elbowVolts = 0;
		arm.stopMotor();
		elbow.stopMotor();
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("DoubleArmMotor", arm));
		hardware.add(new SelfCheckingTalonFX("DoubleElbowMotor", elbow));
		return hardware;
	}
}
