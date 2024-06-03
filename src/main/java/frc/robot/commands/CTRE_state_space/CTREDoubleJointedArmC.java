package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataHandler;
import frc.robot.subsystems.CTRE_state_space.CTREDoubleJointedArmS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;
import java.util.List;

public class CTREDoubleJointedArmC extends Command {
		private final CTREDoubleJointedArmS armS;
		public static List<Double> voltages;
	public CTREDoubleJointedArmC(CTREDoubleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void execute() {
		if (CTRESpaceConstants.Controls.gotoUpRight.getAsBoolean()){
			DataHandler.logData("point2","doubleJointSetpoint");
		}else if (CTRESpaceConstants.Controls.gotoUpLeft.getAsBoolean()){
			DataHandler.logData("point1","doubleJointSetpoint");
		}
		System.out.println(voltages);
		if (voltages != null){
			armS.setMotors(voltages.get(0), voltages.get(1));
		}
	}

}	
