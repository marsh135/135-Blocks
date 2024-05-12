package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangS;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;

public class HangMacroC extends Command {
	private final HangS hangS;
	public static boolean isFinished;
	public double deadband, desHeight;
	public static PIDController leftHangController = new PIDController(
			.04, 0, 0),
			rightHangController = new PIDController(.04, 0, 0);

	public HangMacroC(HangS hang, double desHeight) {
		this.hangS = hang;
		this.desHeight = desHeight;
		addRequirements(hangS);
		deadband = 1;
	}

	@Override
	public void initialize() { isFinished = false; }

	@Override
	public void execute() {
		if ((Math.abs(leftHangController.getPositionError()) < deadband
			 && Math.abs(rightHangController.getPositionError()) < deadband)
			 || (Math.abs(RobotContainer.manipController.getLeftY())) > .1) {
			isFinished = true;
		}
		double leftOutput = leftHangController.calculate(
				HangS.leftHangEncoder.getPosition(), desHeight);
		double rightOutput = rightHangController.calculate(
				HangS.rightHangEncoder.getPosition(),
				desHeight);
		if (leftOutput > 1)
			leftOutput = 1;
		if (rightOutput > 1)
			rightOutput = 1;
		hangS.setHangMotors(leftOutput, rightOutput);
		// SmartDashboard.putNumber("Left Hang Output", leftOutput);
		//SmartDashboard.putNumber("Right Hang Output", rightOutput);
	}

	@Override
	public void end(boolean interrupted) {
		hangS.setHangMotors(0, 0);
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
