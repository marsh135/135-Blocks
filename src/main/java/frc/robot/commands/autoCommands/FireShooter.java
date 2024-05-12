package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.OutakeS;
import frc.robot.utils.SimShootNote;

public class FireShooter extends Command {
	private final OutakeS outakeS;
	private int desRPM;
	private final double time = 0.25;
	private boolean isFinished = false;
	private final Timer timer = new Timer();

	public FireShooter(OutakeS outakeS) {
		this.outakeS = outakeS;
		addRequirements(outakeS);
	}

	@Override
	public void initialize() {
		//resets timer
		isFinished = false;
		timer.reset();
	}

	@Override
	public void execute() {
		if (AutonIntake.allClear) {
			if (CameraS
					.getDistanceFromSpeakerUsingRobotPose() > 4.5) {
				desRPM = 6000;
			} else if (CameraS
					.getDistanceFromSpeakerUsingRobotPose() > 2.4) {
				desRPM = 4750;
			} else {
				desRPM = 3300;
			}
			outakeS.setIndividualFlywheelSpeeds(desRPM,
					desRPM);
			//if the timer hasnt reached the time, essentially uses a pid loop with a feedforward constant (desired velocity/max velocity) to set the motor speed as a percentage
			if (timer.get() >= time) {
				isFinished = true;
			}
			SmartDashboard.putNumber("Flywheel Top",
					OutakeS.topFlywheelEncoder.getVelocity());
			SmartDashboard.putNumber("Flywheel Bottom ",
					OutakeS.bottomFlywheelEncoder.getVelocity());
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (Robot.isSimulation()) {
			SimShootNote.shoot();
		}
		timer.stop();
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
