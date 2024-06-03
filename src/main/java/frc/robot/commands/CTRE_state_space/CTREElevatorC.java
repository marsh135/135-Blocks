package frc.robot.commands.CTRE_state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_state_space.CTREElevatorS;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTREElevatorC extends Command {
	private final CTREElevatorS elevatorS;

	public CTREElevatorC(CTREElevatorS elevatorS) {
		this.elevatorS = elevatorS;
		addRequirements(elevatorS);
	}

	@Override
	public void initialize() {
		//do nothing, armS handles startup.
	}

	@Override
	public void execute() {
		double elevatorSpeed = 0, elevatorPos = CTREElevatorS.getSetpoint();
		double desSpeed = -RobotContainer.manipController.getLeftY();
		if (CTRESpaceConstants.Controls.go2ftButton.getAsBoolean()) {
			elevatorPos = Units.feetToMeters(2);
		} else if (CTRESpaceConstants.Controls.go0ftButton.getAsBoolean()) {
			elevatorPos = Units.feetToMeters(0);
		}
		if (Math.abs(desSpeed) > CTRESpaceConstants.Controls.kArmDeadband) {
			if (elevatorPos >= CTRESpaceConstants.Elevator.maxPosition && desSpeed > 0) {
				elevatorPos = CTRESpaceConstants.Elevator.maxPosition;
			} else if (elevatorPos <= CTRESpaceConstants.Elevator.startingPosition
					&& desSpeed < 0) {
				elevatorPos = CTRESpaceConstants.Elevator.startingPosition;
			} else {
				elevatorSpeed = desSpeed * CTRESpaceConstants.Elevator.maxAcceleration
						* CTRESpaceConstants.Controls.elevatorMoveSpeed;
				elevatorPos += (elevatorSpeed * CTREElevatorS.dtSeconds); //add to our current position 20 MS of that accel
			}
		}
		if (elevatorSpeed == 0) {
			elevatorS.moveElevator(elevatorS.createState(elevatorPos));
		} else {
			elevatorS.moveElevator(elevatorS.createState(elevatorPos, elevatorSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}