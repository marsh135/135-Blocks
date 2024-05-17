package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
		double armSpeed = 0, armPos = 0;
		//implement a way to take a percentage output (0 - 1 ) to a velocity for the deploy intake. 
		// Predict .02seconds later to the current position with the given velocity
		// Feed that into the state position
		// use the velocity given for state velocity 
		// Remember, the state uses DEGREES.
		//intakeS.deployIntake(deployIntakeSpeed * 1);
		if (StateSpaceConstants.Controls.go45Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(45);
		}
		if (StateSpaceConstants.Controls.go0Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(0);
		}
		if (Math.abs(armSpeed) < StateSpaceConstants.Controls.kArmDeadband) {
			armSpeed = 0;
		}
		if (Math.abs(armPos) < StateSpaceConstants.Controls.kArmDeadband) {
			armSpeed = 0;
		}
		armS.deployIntake(armS.createState(armPos));
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
