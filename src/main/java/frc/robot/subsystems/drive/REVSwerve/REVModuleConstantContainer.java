package frc.robot.subsystems.drive.REVSwerve;
import frc.robot.MotorConstantContainer;

public class REVModuleConstantContainer {
	private static int[] moduleConstantContainerInts;
	private static boolean[] moduleConstantContainerBools;
	private static MotorConstantContainer[] moduleMotorConstants;
	/**
	 * Wrapper class designed to hold all the constants for a rev swerve module in the 135-blocks framework
	 * @param driveMotorID the CAN ID of the drive motor
	 * @param turningMotorID the CAN ID of the turning motor
	 * @param driveMotorReversed whether the drive motor is reversed (positive percent/voltage makes it go backwards)
	 * @param turningMotorReversed whether the turning motor is reversed (positive percent/voltage makes it go backwards)
	 * @param absoluteEncoderOffset the offset of the absolute encoder (when the module is zeroed, what value is output)
	 * @param absoluteEncoderReversed whether the absolute encoder is reversed
	 * @param driveMotorConstantContainer motor constant container for the drive motor @see motorConstantContainer
	 * @param turningMotorConstantContainer motor constant container for the turning motor @see motorConstantContainer
	 */
	public REVModuleConstantContainer(int driveMotorID, int turningMotorID,
			boolean driveMotorReversed, boolean turningMotorReversed,
			double absoluteEncoderOffset,
			boolean absoluteEncoderReversed,
			MotorConstantContainer driveMotorConstantContainer,
			MotorConstantContainer turningMotorConstantContainer){
				moduleConstantContainerInts = new int[]{driveMotorID, turningMotorID};
				moduleConstantContainerBools = new boolean[]{driveMotorReversed,turningMotorReversed,absoluteEncoderReversed};
				moduleMotorConstants = new MotorConstantContainer[]{driveMotorConstantContainer,turningMotorConstantContainer};
			}
	public double getDriveMotorID(){
		return moduleConstantContainerInts[0];
	}
	public double getTurningMotorID(){
		return moduleConstantContainerInts[1];
	}
	public boolean getDriveMotorReversed(){
		return moduleConstantContainerBools[0];
	}
	public boolean getTurningMotorReversed(){
		return moduleConstantContainerBools[1];
	}
	public boolean getAbsoluteEncoderReversed(){
		return moduleConstantContainerBools[2];
	}
	public MotorConstantContainer getDriveMotorConstantContainer(){
		return moduleMotorConstants[0];
	}
	public MotorConstantContainer getTurningMotorConstantContainer(){
		return moduleMotorConstants[1];
	}
			

}
