package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_state_space.CTREArmS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTREArmC extends Command {
	private final CTREArmS armS;

	public CTREArmC(CTREArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		//do nothing, armS handles startup.
	}

	@Override
	public void execute() {
		double armSpeed = 0, armPos = CTREArmS.getSetpoint();
		double desSpeed = -RobotContainer.manipController.getRightY();
		if (CTRESpaceConstants.Controls.go45Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(45);
		} else if (CTRESpaceConstants.Controls.go0Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(0);
		}
		if (Math.abs(desSpeed) > CTRESpaceConstants.Controls.kArmDeadband) {
			if (armPos >= CTRESpaceConstants.Arm.maxPosition && desSpeed > 0) {
				armPos = CTRESpaceConstants.Arm.maxPosition;
			} else if (armPos <= CTRESpaceConstants.Arm.startingPosition
					&& desSpeed < 0) {
				armPos = CTRESpaceConstants.Arm.startingPosition;
			} else {
				armSpeed = desSpeed * CTRESpaceConstants.Arm.maxAcceleration
						* CTRESpaceConstants.Controls.armMoveSpeed;
				armPos += (armSpeed * CTREArmS.dtSeconds); //add to our current position 20 MS of that accel
			}
		}
		if (armSpeed == 0) {
			armS.deployIntake(armS.createState(armPos));
		} else {
			armS.deployIntake(armS.createState(armPos, armSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}