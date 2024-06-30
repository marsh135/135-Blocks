package frc.robot.subsystems.state_space.Flywheel;

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

public class FlywheelIOTalon implements FlywheelIO {
	private double appliedVolts = 0.0;
	private TalonFX flywheel;
	private final StatusSignal<Double> flywheelPosition = flywheel.getPosition();
   private final StatusSignal<Double> flywheelVelocity = flywheel.getVelocity();
   private final StatusSignal<Double> flywheelAppliedVolts = flywheel.getMotorVoltage();
   private final StatusSignal<Double> flywheelCurrent = flywheel.getSupplyCurrent();
	private final StatusSignal<Double> flywheelTemp = flywheel.getDeviceTemp();
	public FlywheelIOTalon(){
		flywheel = new TalonFX(StateSpaceConstants.Flywheel.kMotorID);
		var config = new TalonFXConfiguration();
    	config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.Flywheel.currentLimit;
    	config.CurrentLimits.SupplyCurrentLimitEnable = true;
    	config.MotorOutput.NeutralMode = StateSpaceConstants.Flywheel.isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.Flywheel.inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    	flywheel.getConfigurator().apply(config);
 		BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelTemp);
      flywheel.optimizeBusUtilization();
	}
	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		BaseStatusSignal.refreshAll(
			flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelTemp);
		flywheel.setVoltage(appliedVolts);
		inputs.appliedVolts = flywheelAppliedVolts.getValue();
		inputs.positionRad = Units.rotationsToRadians(flywheelPosition.getValue() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.velocityRadPerSec = Units.rotationsToRadians(flywheelVelocity.getValue() * StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.currentAmps = new double[] {flywheelCurrent.getValue()};
		inputs.flywheelTemp = flywheelTemp.getValue();
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
		hardware.add(new SelfCheckingTalonFX("Flywheel", flywheel));
		return hardware;
	}
}
