package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_state_space.CTRESingleJointedArmS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTRESingleJointedArmC extends Command {
	private final CTRESingleJointedArmS armS;
	double armPos = 0;
	public CTRESingleJointedArmC(CTRESingleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		CTRESpaceConstants.Controls.go45Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(45);
							armS.deployArm(armS.createState(armPos));
						}));
		CTRESpaceConstants.Controls.go0Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(0);
							armS.deployArm(armS.createState(armPos));
						}));
	}

	@Override
	public void execute() {
		double armSpeed = 0, desSpeed = -RobotContainer.manipController.getRightY();
		armPos = CTRESingleJointedArmS.getSetpoint();
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
			armS.deployArm(armS.createState(armPos));
		} else {
			armS.deployArm(armS.createState(armPos, armSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}