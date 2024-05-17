package frc.robot.utils.state_space;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class StateSpaceConstants {
	public static boolean debug = true;

	public class Controls {
		/* Enter any non-button controls here.
		 * Left trigger is used for RPM speed. 0-0 1-7100.
		 */
		public static double kDeadband = 0.1;
		public static JoystickButton setButton = new JoystickButton(
				RobotContainer.manipController, 3);
	}

	public class Flywheel {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 20, maxRPM = 7100;
		public static double kFlywheelP = 0, kFlywheelSVolts = 0,
				kFlywheelVVoltSecondsPerRotation = 0,
				kFlywheelAVoltSecondsSquaredPerRotation = 0, m_KalmanModel = 3,
				m_KalmanEncoder = 0.01, m_LQRQelms = 1, m_LQRVolts = 12,
				flywheelGearing = 1.5;
	}

	public class Arm {
		public static int kMotorID = 30;
		public static double kArmP = 0, kArmD = 0, //must have position set in SysId
				kArmSVolts = 0, kArmVVoltSecondsPerRotation = 0,
				kArmAVoltSecondsSquaredPerRotation = 0;
	}
}
