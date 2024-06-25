package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.DataHandler;
import frc.robot.subsystems.CTRE_state_space.CTREDoubleJointedArmS;
import frc.robot.utils.CTRE_state_space.StateSpaceConstants;

public class CTREDoubleJointedArmC extends Command {
	@SuppressWarnings("unused")
	private final CTREDoubleJointedArmS armS;

	public CTREDoubleJointedArmC(CTREDoubleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.gotoUpRight
				.onTrue(new InstantCommand(() -> DataHandler.logData(
						StateSpaceConstants.DoubleJointedArm.macroTopRight,
						"DoubleJointSetpoint")));
		StateSpaceConstants.Controls.gotoUpLeft
						.onTrue(new InstantCommand(() -> DataHandler.logData(
								StateSpaceConstants.DoubleJointedArm.macroTopLeft,
								"DoubleJointSetpoint")));
	}

	@Override
	public void execute() {
		
	}
}
