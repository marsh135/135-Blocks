package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeS;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

// Heads up! It's unimportant, but if the driver shoots a note in sim, picks up
// a note BEFORE the shot one is at the amp, and then attempts to shoot the new
// note BEFORE the old shot one is at the amp, it will desync
// the array by 1, which will crash the program at the last note on the field.
// I'm not fixing this, because it'd require a lot of time, and is a VERY
// one-off scenario.
// To be exact, the "one-off scenario" is TWO NOTE CYCLE IN ONE SECOND. It
// doesn't happen. Technically, it could be fixed by upping shot speed, but we
// like to see the notes fly.
public class SimShootNote {
	//Speaker translations
	private static final Translation3d blueSpeaker = new Translation3d(0.225,5.55, 2.1);
	private static final Translation3d redSpeaker = new Translation3d(16.317,5.55, 2.1);
	private static final double shotSpeed = 15;
	private static final double intakeSpeed = 3;
	private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
	//Launcher position compared to the robot
	static Transform3d launcherTransform = new Transform3d(0.292, 0, 0.1225,
			new Rotation3d(0, -Units.degreesToRadians(IntakeS.getDistance()
					- Constants.IntakeConstants.intakeOffset + 8), 0.0));

	//Returns the robot pose
	public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
		robotPoseSupplier = supplier;
	}

	public static ArrayList<Pose3d> currentNotes = new ArrayList<Pose3d>();
	static Pose3d heldNotePos = new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
	static public boolean firstIntakeCycle = true, hasNote = false, firstOutake = true;

	public static void resetNotes() {
		currentNotes.clear();
		for (int i = 0; i < Constants.DriveSimConstants.fieldNoteTranslations.length; i++) {
			currentNotes.add(Constants.DriveSimConstants.fieldNoteTranslations[i]);
		}
		if (DriverStation.isAutonomous()) {
			currentNotes.add(0, heldNotePos);
			hasNote = true;
		}
	}

	public static void intake(int index) {
		new ScheduleCommand( // Branch off and exit immediately
				Commands.defer(() -> {
					launcherTransform = new Transform3d(0.3, 0, 0.36,
							new Rotation3d(0,
									-Units.degreesToRadians(IntakeS.getDistance() - Constants.IntakeConstants.intakeOffset + 8),
									0.0));
					final Pose3d startPose = currentNotes.get(index);
					final Pose3d endPose = new Pose3d(robotPoseSupplier.get())
							.transformBy(launcherTransform);
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation()) / intakeSpeed;
					final Timer timer = new Timer();
					timer.start();
					firstIntakeCycle = true;
					if (!hasNote) {
						currentNotes.remove(index);
						return Commands.run(() -> {
							if (SimShootNote.firstIntakeCycle) {
								currentNotes.add(0, startPose.interpolate(endPose, timer.get() / duration));
								SimShootNote.firstIntakeCycle = false;
							} else {
								currentNotes.remove(0);
								currentNotes.add(0, startPose.interpolate(endPose, timer.get() / duration));
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> {
							//currentNotes.remove(0);
							hasNote = true;
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true)).schedule();
	}

	public static Pose3d[] getState() {
		Pose3d[] translator = new Pose3d[currentNotes.size()];
		for (int i = 0; i < currentNotes.size(); i++) {
			translator[i] = currentNotes.get(i);
		}
		return translator;
	}

	public static void updateStates() {
		Pose3d[] translator = new Pose3d[currentNotes.size()];
		launcherTransform = new Transform3d(0.3, 0, 0.36,
				new Rotation3d(0, -Units.degreesToRadians(IntakeS.getDistance()- Constants.IntakeConstants.intakeOffset + 8),
				 0.0));
		heldNotePos = new Pose3d(robotPoseSupplier.get())
				.transformBy(launcherTransform);
		for (int i = 0; i < currentNotes.size(); i++) {
			if (i == 0 && hasNote) {
				translator[i] = heldNotePos;
			} else {
				translator[i] = currentNotes.get(i);
			}
		}
		Logger.recordOutput("NoteVisualizer", translator);
	}

	public static Command shoot() {
		return new ScheduleCommand( // Branch off and exit immediately
				Commands.defer(() -> {
					//Initial starting point
					final Pose3d startPose = heldNotePos;
					final boolean isRed = DriverStation.getAlliance().isPresent()
							&& DriverStation.getAlliance().get().equals(Alliance.Red);
					final Pose3d endPose = new Pose3d(
							isRed ? redSpeaker : blueSpeaker, startPose.getRotation());
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation()) / shotSpeed;
					final Timer timer = new Timer();
					timer.start();
					firstOutake = true;
					if (hasNote) {
						hasNote = false;
						int indexChanger;
						if (currentNotes.size() == 1) {
							indexChanger = 1;
						} else {
							indexChanger = 2;
						}
						return Commands.run(() -> 
						{
							if (SimShootNote.firstOutake && indexChanger == 2) {
								currentNotes.remove(0);
								currentNotes.add(currentNotes.size() - 1, startPose
										.interpolate(endPose, timer.get() / duration));
								SimShootNote.firstOutake = false;
							} else {
								currentNotes.remove(currentNotes.size() - indexChanger);
								if (indexChanger == 2) {
									currentNotes.add(currentNotes.size() - 1, startPose
											.interpolate(endPose, timer.get() / duration));
								} else {
									currentNotes.add(currentNotes.size(), startPose
											.interpolate(endPose, timer.get() / duration));
								}
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> 
						{
							currentNotes.remove(currentNotes.size() - indexChanger);
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true));
	}
}