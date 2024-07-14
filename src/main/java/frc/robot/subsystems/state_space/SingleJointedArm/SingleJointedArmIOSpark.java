package frc.robot.subsystems.state_space.SingleJointedArm;

import java.util.ArrayList;
import java.util.List;

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

public class SingleJointedArmIOSpark implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private CANSparkBase arm;
	private RelativeEncoder encoder;

	public SingleJointedArmIOSpark() {
		if (StateSpaceConstants.SingleJointedArm.motorVendor == MotorVendor.NEO_SPARK_MAX) {
			arm = new CANSparkMax(StateSpaceConstants.SingleJointedArm.kMotorID,
					MotorType.kBrushless);
		} else {
			arm = new CANSparkFlex(StateSpaceConstants.SingleJointedArm.kMotorID,
					MotorType.kBrushless);
		}
		arm.enableVoltageCompensation(12);
		arm.setIdleMode(
				StateSpaceConstants.SingleJointedArm.isBrake ? IdleMode.kBrake
						: IdleMode.kCoast);
		arm.setCANTimeout(250);
		arm.setInverted(StateSpaceConstants.SingleJointedArm.inverted);
		arm.setSmartCurrentLimit(
				StateSpaceConstants.SingleJointedArm.currentLimit);
		encoder = arm.getEncoder();
		arm.burnFlash();
	}

	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		arm.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.SingleJointedArm.armGearing);;
		inputs.armTemp = arm.getMotorTemperature();
		inputs.velocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.SingleJointedArm.armGearing);
		inputs.currentAmps = new double[] { arm.getOutputCurrent()
		};
	}

	@Override
	public void setVoltage(double volts) { arm.setVoltage(volts); }

	@Override
	/** Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop() {
		setVoltage(0);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("SingleArm", arm));
		return hardware;
	}
}