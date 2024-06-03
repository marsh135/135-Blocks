package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_state_space.CTRESingleJointedArmS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTRESingleJointedArmC extends Command {
	private final CTRESingleJointedArmS armS;

	public CTRESingleJointedArmC(CTRESingleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		//do nothing, armS handles startup.
	}

	@Override
	public void execute() {
		double armSpeed = 0, armPos = CTRESingleJointedArmS.getSetpoint();
		double desSpeed = -RobotContainer.manipController.getRightY();
		if (CTRESpaceConstants.Controls.go45Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(45);
		} else if (CTRESpaceConstants.Controls.go0Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(0);
		}
		if (Math.abs(desSpeed) > CTRESpaceConstants.Controls.kArmDeadband) {
			if (armPos >= CTRESpaceConstants.SingleJointedArm.maxPosition && desSpeed > 0) {
				armPos = CTRESpaceConstants.SingleJointedArm.maxPosition;
			} else if (armPos <= CTRESpaceConstants.SingleJointedArm.startingPosition
					&& desSpeed < 0) {
				armPos = CTRESpaceConstants.SingleJointedArm.startingPosition;
			} else {
				armSpeed = desSpeed * CTRESpaceConstants.SingleJointedArm.maxAcceleration
						* CTRESpaceConstants.Controls.armMoveSpeed;
				armPos += (armSpeed * CTRESingleJointedArmS.dtSeconds); //add to our current position 20 MS of that accel
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