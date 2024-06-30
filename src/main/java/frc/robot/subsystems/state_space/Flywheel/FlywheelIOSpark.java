package frc.robot.subsystems.state_space.Flywheel;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.drive.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelIOSpark implements FlywheelIO {
	private double appliedVolts = 0.0;
	private CANSparkBase flywheel;
	private RelativeEncoder encoder;

	public FlywheelIOSpark(){
		if (StateSpaceConstants.Flywheel.motorVendor == MotorVendor.NEO_SPARK_MAX){
			flywheel = new CANSparkMax(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
		}else{
			flywheel = new CANSparkFlex(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
		}
		flywheel.enableVoltageCompensation(12);
		flywheel.setIdleMode(StateSpaceConstants.Flywheel.isBrake ? IdleMode.kBrake : IdleMode.kCoast);
		flywheel.setCANTimeout(250);
		flywheel.setInverted(StateSpaceConstants.Flywheel.inverted);
		flywheel.setSmartCurrentLimit(StateSpaceConstants.Flywheel.currentLimit);
		encoder = flywheel.getEncoder();
		flywheel.burnFlash();
	}
	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		flywheel.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.currentAmps = new double[] {flywheel.getOutputCurrent()};
		inputs.flywheelTemp = flywheel.getMotorTemperature();
	}
	@Override
	public void setVoltage(double volts){
		appliedVolts = volts;
	}
	@Override
	/**Stop the flywheel by telling it to go to 0 rpm. */
	public void stop(){
		appliedVolts = 0;
		flywheel.stopMotor();
	}
	@Override
	public List<SelfChecking> getSelfCheckingHardware(){
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Flywheel", flywheel));
		return hardware;
	}
}
