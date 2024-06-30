package frc.robot.subsystems.state_space.SingleJointedArm;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingTalonFX;
import frc.robot.utils.state_space.StateSpaceConstants;

public class SingleJointedArmIOTalon implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private TalonFX arm;
	private final StatusSignal<Double> armPosition = arm.getPosition();
   private final StatusSignal<Double> armVelocity = arm.getVelocity();
   private final StatusSignal<Double> armAppliedVolts = arm.getMotorVoltage();
   private final StatusSignal<Double> armCurrent = arm.getSupplyCurrent();
	private final StatusSignal<Double> armTemp = arm.getDeviceTemp();
public SingleJointedArmIOTalon(){
		arm = new TalonFX(StateSpaceConstants.SingleJointedArm.kMotorID);
		var config = new TalonFXConfiguration();
    	config.CurrentLimits.SupplyCurrentLimit = StateSpaceConstants.SingleJointedArm.currentLimit;
    	config.CurrentLimits.SupplyCurrentLimitEnable = true;
    	config.MotorOutput.NeutralMode = StateSpaceConstants.SingleJointedArm.isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.SingleJointedArm.inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    	arm.getConfigurator().apply(config);
 		BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armPosition, armVelocity, armAppliedVolts, armCurrent, armTemp);
      arm.optimizeBusUtilization();
	}
	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		BaseStatusSignal.refreshAll(
			armPosition, armVelocity, armAppliedVolts, armCurrent, armTemp);
		
		arm.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.armTemp = armTemp.getValue();
		inputs.positionRad = Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(armPosition, armVelocity, .2) * StateSpaceConstants.SingleJointedArm.armGearing);;
		inputs.velocityRadPerSec =  Units.rotationsToRadians(armVelocity.getValue() * StateSpaceConstants.SingleJointedArm.armGearing);
		inputs.currentAmps = new double[] {armCurrent.getValue()}; 
	}
	@Override
	public void setVoltage(double volts){
		appliedVolts = volts;
	}
	@Override
	/**Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop(){
		setVoltage(0);
	}
		@Override
	public List<SelfChecking> getSelfCheckingHardware(){
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("SingleArm", arm));
		return hardware;
	}
}