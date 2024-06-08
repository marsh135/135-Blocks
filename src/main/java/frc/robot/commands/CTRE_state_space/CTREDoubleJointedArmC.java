package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
	public void initialize() {
		CTRESpaceConstants.Controls.gotoUpRight
				.onTrue(new InstantCommand(() -> DataHandler.logData(
						CTRESpaceConstants.DoubleJointedArm.macroTopRight,
						"DoubleJointSetpoint")));
		CTRESpaceConstants.Controls.gotoUpLeft
						.onTrue(new InstantCommand(() -> DataHandler.logData(
								CTRESpaceConstants.DoubleJointedArm.macroTopLeft,
								"DoubleJointSetpoint")));
	}

	@Override
	public void execute() {
		if (voltages != null) {
			armS.setMotors(voltages.get(0), voltages.get(1));
		}
	}
}
