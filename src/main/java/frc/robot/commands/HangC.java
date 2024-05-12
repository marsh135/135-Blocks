package frc.robot.commands;

// import
// edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.HangS;

public class HangC extends Command {
	private final HangS hangS;
	//private PIDController hangPidController = new PIDController(0.01, 0, 0);

	public HangC(HangS hangS) {
		this.hangS = hangS;
		addRequirements(hangS);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double output = RobotContainer.manipController
				.getLeftY() * -1;
		output = Math.abs(output) >= 0.2 ? output : 0;
		//double hangAdjustment = hangPidController.calculate(SwerveS.getTilt(), 0);
		double hangAdjustment = 0;
		double leftOutput = output + hangAdjustment;
		double rightOutput = output - hangAdjustment;
		hangS.setHangMotors(leftOutput, rightOutput);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
