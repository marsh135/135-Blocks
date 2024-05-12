package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeS;

public class MoveIntake extends Command {
	//variable declaration, new timer made, requirements added
	boolean isFinished = false;
	IntakeS intakeS;
	Timer timer = new Timer();

	public MoveIntake(IntakeS intakeS) {
		this.intakeS = intakeS;
		addRequirements(intakeS);
	}

	public void initialize() {
		//resets and starts the timer upon command being called
		isFinished = false;
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		//intake deployed at full power for a certain amount of time
		if (Robot.isSimulation()) {
			if (timer.get() > .25) {
				isFinished = true;
			}
		} else {
			intakeS.deployIntake(intakeS.outsideBotState());
		}
		if (intakeS.isAtState()) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
	}

	@Override
	public boolean isFinished() { return isFinished; }
}