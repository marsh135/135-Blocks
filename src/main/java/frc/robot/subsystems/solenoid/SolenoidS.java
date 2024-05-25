package frc.robot.subsystems.solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.solenoid.SolenoidConstants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SolenoidS extends SubsystemBase {
	public static Solenoid singleSolenoid;
	public static DoubleSolenoid doubleSolenoid;

	/**
	 * Creates a new subsystem to handle solenoid sims. Holds example for a
	 * single-action and a double-action solenoid. Also works in sim
	 */
	public SolenoidS() {
		singleSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
				SolenoidConstants.singleActingSolenoidChannel);
		doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
				SolenoidConstants.doubleActingSolenoidChannelForward,
				SolenoidConstants.doubleActingSolenoidChannelBackward);
		
		singleSolenoid.setPulseDuration(0);
		singleSolenoid.startPulse();
	}
	/**
	 * Sets the state of the single solenoid
	 * @param on whether it is on or off (off is false)
	 */
	public static void setSingleSolenoid(boolean on){
		singleSolenoid.set(on);
	}
	/**
	 * @return the state of the single solenoid as a value (literally read the line above goober)
	 */
	public static boolean getSingleSolenoid(){
		return singleSolenoid.get();
	}
	/**
	 * Set the state of the double solenoid
	 * @param state the desired state of the double solenoid (Value.kForward, Value.kReverse, Value.kOff)
	 */
	public static void setDoubleSolenoid(DoubleSolenoid.Value state){
		doubleSolenoid.set(state);
	}
	/**
	 * 
	 * @return the value of the solenoid (Either Value.kFoward, Value.kReverse, Value.kOff)
	 */
	public static DoubleSolenoid.Value getDoubleSolenoidState(){
		return doubleSolenoid.get();
	}	
	@Override
	public void periodic(){
		
	}
}
