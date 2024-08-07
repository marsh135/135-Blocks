package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.selfCheck.SelfChecking;

public interface ColorSensorIO {
		@AutoLog
	
	public static class ColorSensorIOInputs{
		public Color colorOutput;
		public double proximityCentimeters;
	}

	public default void updateInputs(ColorSensorIOInputs inputs) {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}

