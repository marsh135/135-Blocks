// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.HangConstants;
import frc.robot.commands.HangC;
import frc.robot.commands.IntakeC;
import frc.robot.commands.OutakeC;
import frc.robot.commands.SetAngle;
import frc.robot.commands.SwerveC;
import frc.robot.commands.VariableAngle;
import frc.robot.commands.VariableSpeed;
import frc.robot.commands.autoCommands.AutoLock;
import frc.robot.commands.autoCommands.AutonIntake;
import frc.robot.commands.autoCommands.FireShooter;
import frc.robot.commands.autoCommands.MoveIntake;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.HangS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.subsystems.SwerveS;
import frc.robot.utils.SimShootNote;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.HangMacroC;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.LEDStripS;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveS swerveS = new SwerveS();
	private final IntakeS intakeS = new IntakeS();
	private final OutakeS outakeS = new OutakeS();
	private final HangS hangS = new HangS();
	@SuppressWarnings("unused") //only periodic ledStrip is used, so don't care.
	private final LEDStripS ledStripS = new LEDStripS();
	@SuppressWarnings("unused") //only periodic camera updates is used, so don't care.
	private final CameraS cameraS = new CameraS();
	private final SendableChooser<Command> autoChooser;

	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	static JoystickButton 
	aButtonDrive = new JoystickButton(driveController, 1),
	bButtonDrive = new JoystickButton(driveController, 2),
	xButtonDrive = new JoystickButton(driveController, 3),
	yButtonDrive = new JoystickButton(driveController, 4),
	selectButtonDrive = new JoystickButton(driveController, 7),
	startButtonDrive = new JoystickButton(driveController, 8),
	bButtonManip = new JoystickButton(manipController, 2);

	//POVButton povUpManip = new POVButton(driveController, 0);
	POVButton 
	povRightDrive = new POVButton(driveController, 90),
	 povDownDrive = new POVButton(driveController, 180),
	 povLeftDrive = new POVButton(driveController, 270),
	 povUpDrive = new POVButton(driveController, 0);

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		swerveS.setDefaultCommand(new SwerveC(swerveS));
		intakeS.setDefaultCommand(new IntakeC(intakeS));
		outakeS.setDefaultCommand(new OutakeC(outakeS));
		hangS.setDefaultCommand(new HangC(hangS));
		NamedCommands.registerCommand("DeployIntake",
			   new MoveIntake(intakeS));
		NamedCommands.registerCommand("Lock Onto April Tags",
				new AutoLock(swerveS));
		NamedCommands.registerCommand("IntakeNote",
				new AutonIntake(intakeS, swerveS));
		if (Robot.isSimulation()){
			NamedCommands.registerCommand("Shoot From Anywhere",
				new SequentialCommandGroup(
						new VariableAngle(intakeS, outakeS, true),
						SimShootNote.shoot()
				));
		}else{
			NamedCommands.registerCommand("Shoot From Anywhere",
				new VariableAngle(intakeS, outakeS, true)
			);
		}
		NamedCommands.registerCommand("Hold Outake Ready",
				new FireShooter(outakeS));

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		aButtonDrive.and(isDriving()).onTrue(swerveS.toggleAutoLockCommand());
		if (Robot.isSimulation()){
			xButtonDrive.and(isDriving()).onTrue(SimShootNote.shoot());
		}else{
			xButtonDrive.and(isDriving()).onTrue(
				new InstantCommand(() -> swerveS.zeroHeading()));
		}
		yButtonDrive.and(isDriving()).onTrue(
				new VariableSpeed(intakeS, outakeS, false));
		bButtonManip.and(isDriving()).onTrue(
				new SetAngle(intakeS, outakeS, 13));
		bButtonDrive.whileTrue(
				new AutonIntake(intakeS, swerveS));
		//System.out.println(SysIdRoutine.Direction.kReverse);
		//System.out.println(SysIdRoutine.Direction.kForward);
		//outake Tests
		startButtonDrive.and(aButtonDrive).whileTrue(
				intakeS.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(bButtonManip).whileTrue(
				intakeS.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		//startButtonDrive.and(xButtonDrive).whileTrue(intakeS.sysIdDynamic(SysIdRoutine.Direction.kForward));
		//startButtonDrive.and(yButtonDrive).whileTrue(intakeS.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		//swerve DRIVE tests
		startButtonDrive.and(povUpDrive).whileTrue(
				swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povRightDrive).whileTrue(
				swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
		startButtonDrive.and(povDownDrive).whileTrue(
				swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povLeftDrive).whileTrue(
				swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
		//swerve TURNING tests
		selectButtonDrive.and(povUpDrive).whileTrue(
				swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kForward));
		selectButtonDrive.and(povRightDrive).whileTrue(
				swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kReverse));
		selectButtonDrive.and(povDownDrive).whileTrue(
				swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kForward));
		selectButtonDrive.and(povLeftDrive).whileTrue(
				swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kReverse));
		//manipController.y().and(manipController.start().negate()).onTrue(new VariableSpeed(intakeS, outakeS, false));
		//manipController.b().and(manipController.start().negate()).onTrue(new SetAngle(intakeS, outakeS, 13));
		povUpDrive.and(isDriving())
				.whileTrue(new HangMacroC(hangS, HangConstants.upperHookHeight))
				.whileTrue(new SetAngle(intakeS, outakeS, 27));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public static BooleanSupplier isDriving() {
		BooleanSupplier returnVal;
		if (startButtonDrive.getAsBoolean() || selectButtonDrive.getAsBoolean()) {
			returnVal = () -> false;
			return returnVal; //currently doing a test
		}
		returnVal = () -> true;
		return returnVal;
	}
}