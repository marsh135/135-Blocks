package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.HangConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangS extends SubsystemBase {
	//static double hangState = 0;
	//hang motors
	public static CANSparkMax leftHang = new CANSparkMax(
			HangConstants.leftHangID, MotorType.kBrushless);
	public static CANSparkMax rightHang = new CANSparkMax(
			HangConstants.rightHangID, MotorType.kBrushless);
	public static RelativeEncoder leftHangEncoder, rightHangEncoder;

	//sets hang idle mode & saves it
	public HangS() {
		leftHang.setIdleMode(IdleMode.kBrake);
		rightHang.setIdleMode(IdleMode.kBrake);
		leftHang.setInverted(HangConstants.leftHangReversed);
		rightHang.setInverted(HangConstants.rightHangReversed);
		leftHangEncoder = leftHang.getEncoder();
		rightHangEncoder = rightHang.getEncoder();
		leftHang.burnFlash();
		rightHang.burnFlash();
	}

	@Override
	public void periodic() {
		//SmartDashboard.putNumber("Left Hang", leftHangEncoder.getPosition());
		//SmartDashboard.putNumber("Right Hang", rightHangEncoder.getPosition());
		//trying to hang
		/*if((leftHangEncoder.getVelocity() > 15) && (rightHangEncoder.getVelocity() > 15) && (SwerveS.getZAccel() > 0)){
		    hangState = 1;
		}
		//hang done 
		else if (SwerveS.getZDistance() > 10){
		    hangState = 2;
		}
		else{
		    hangState = 0;
		}*/
	}

	public void setHangMotors(double leftOutput, double rightOutput) {
		if (leftOutput < 0 && HangS.leftHangEncoder
				.getPosition() < Constants.HangConstants.hangLowerSoftStop) {
			leftOutput = 0;
		} else if (leftOutput > 0 && HangS.leftHangEncoder
				.getPosition() > Constants.HangConstants.hangUpperSoftStop) {
			leftOutput = 0;
		}
		if (rightOutput < 0 && HangS.rightHangEncoder
				.getPosition() < Constants.HangConstants.hangLowerSoftStop) {
			rightOutput = 0;
		} else if (rightOutput > 0 && HangS.rightHangEncoder
				.getPosition() > Constants.HangConstants.hangUpperSoftStop) {
			rightOutput = 0;
		}
		leftHang.set(leftOutput); //sets the motors to get the controller values
		rightHang.set(rightOutput);
	}
}
