package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.ArmS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ArmC extends Command {
	private final ArmS armS;

	public ArmC(ArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		//do nothing, armS handles startup.
	}

	@Override
	public void execute() {
		double armSpeed = 0, armPos = ArmS.getDistance();
		double desSpeed = RobotContainer.manipController.getRightTriggerAxis();
		if (StateSpaceConstants.Controls.go45Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(45);
		} else if (StateSpaceConstants.Controls.go0Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(0);
		}
		if (Math.abs(desSpeed) > StateSpaceConstants.Controls.kArmDeadband) {
			armSpeed = desSpeed * StateSpaceConstants.Arm.maxAcceleration;
			armPos += (armSpeed * ArmS.dtSeconds); //add to our current position 20 MS of that accel
			//we must first check if anything else is being told before saying "zero speed"
		}
		armS.deployIntake(armS.createState(armPos, armSpeed));
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
