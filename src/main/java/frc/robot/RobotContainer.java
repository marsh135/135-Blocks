// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.auto.BranchAuto;
import frc.robot.commands.auto.SimDefenseBot;
import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.state_space.DoubleJointedArmC;
import frc.robot.commands.state_space.ElevatorC;
import frc.robot.commands.state_space.SingleJointedArmC;
import frc.robot.commands.state_space.FlywheelC;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import frc.robot.subsystems.drive.Mecanum.MecanumIO;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSparkBaseNavx;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSparkBasePigeon;
import frc.robot.subsystems.drive.Mecanum.MecanumIOTalonFXNavx;
import frc.robot.subsystems.drive.Mecanum.MecanumIOTalonFXPigeon;
import frc.robot.subsystems.drive.FastSwerve.ModuleIO;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSparkBaseNavx;
import frc.robot.subsystems.drive.Tank.TankIOSparkBasePigeon;
import frc.robot.subsystems.drive.Tank.TankIOTalonFXNavx;
import frc.robot.subsystems.drive.Tank.TankIOTalonFXPigeon;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIO;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIOSim;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIOTalon;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmS;
import frc.robot.subsystems.state_space.Elevator.ElevatorIO;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOSim;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOTalon;
import frc.robot.subsystems.state_space.Elevator.ElevatorS;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIO;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSpark;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOTalon;
import frc.robot.subsystems.state_space.Flywheel.FlywheelS;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIO;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOSim;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOSpark;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOTalon;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmS;
import frc.robot.utils.RunTest;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.state_space.StateSpaceConstants;
/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	public static FlywheelS flywheelS;
	public static SingleJointedArmS armS;
	public static ElevatorS elevatorS;
	public static DoubleJointedArmS doubleJointedArmS;
	private final SendableChooser<Command> autoChooser;
	//TODO: read PDH
	//static PowerDistribution PDH = Logged
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			//yButtonDrive = new JoystickButton(driveController, 4), //used for Aim/Drive to pose
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	public static int currentTest = 0, currentGamePieceStatus = 0;
	public static String currentPath = "";
	public static Field2d field = new Field2d();
	public static Pose2d opposingBotPose;

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands. y * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
		switch (Constants.currentMode) {
		case REAL:
			switch (DriveConstants.driveType) {
			case SWERVE:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					switch (DriveConstants.gyroType) {
					case NAVX:
						drivetrainS = new Swerve(new GyroIONavX(true),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
						break;
					case PIGEON:
						drivetrainS = new Swerve(new GyroIOPigeon2(true),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
						break;
					default:
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case NAVX:
						drivetrainS = new Swerve(new GyroIONavX(false),
								new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
								new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
						break;
					case PIGEON:
						drivetrainS = new Swerve(new GyroIOPigeon2(true),
								new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
								new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
					default:
						break;
					}
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Tank(new TankIOTalonFXPigeon());
						break;
					case NAVX:
						drivetrainS = new Tank(new TankIOTalonFXNavx());
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Tank(new TankIOSparkBasePigeon());
						break;
					case NAVX:
						drivetrainS = new Tank(new TankIOSparkBaseNavx());
						break;
					}
					break;
				}
				break;
			case MECANUM:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Mecanum(new MecanumIOTalonFXPigeon());
						break;
					case NAVX:
						drivetrainS = new Mecanum(new MecanumIOTalonFXNavx());
						break;
					}
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					switch (DriveConstants.gyroType) {
					case PIGEON:
						drivetrainS = new Mecanum(new MecanumIOSparkBasePigeon());
						break;
					case NAVX:
						drivetrainS = new Mecanum(new MecanumIOSparkBaseNavx());
						break;
					}
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			//Placeholder values
			default:
				throw new IllegalArgumentException(
						"Unknown implementation type, please check DriveConstants.java!");
			}
			
			switch (StateSpaceConstants.Flywheel.motorVendor){
				case CTRE_MOTORS:
				flywheelS = new FlywheelS(new FlywheelIOTalon());

				break;
				default:
				flywheelS = new FlywheelS(new FlywheelIOSpark());
				break;
			}
			switch (StateSpaceConstants.SingleJointedArm.motorVendor){
				case CTRE_MOTORS:
				armS = new SingleJointedArmS(new SingleJointedArmIOTalon());

				break;
				default:
				armS = new SingleJointedArmS(new SingleJointedArmIOSpark());
				break;
			}
			switch (StateSpaceConstants.Elevator.motorVendor){
				case CTRE_MOTORS:
				elevatorS = new ElevatorS(new ElevatorIOTalon());

				break;
				default:
				elevatorS = new ElevatorS(new ElevatorIOSpark());
				break;
			}
			doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOTalon());
			break;
		case SIM:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIOSim(0),
						new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				drivetrainS = new Tank(new TankIOSim());
				break;
			default:
				drivetrainS = new Mecanum(new MecanumIOSim());
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			}
			
			flywheelS = new FlywheelS(new FlywheelIOSim());
			armS = new SingleJointedArmS(new SingleJointedArmIOSim());
			elevatorS = new ElevatorS(new ElevatorIOSim());
			doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOSim());
			break;
		default:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIO() {},
						new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				drivetrainS = new Tank(new TankIO() {});
				break;
			case MECANUM:
				drivetrainS = new Mecanum(new MecanumIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
			}
			flywheelS = new FlywheelS(new FlywheelIO(){});
			armS = new SingleJointedArmS(new SingleJointedArmIO(){});
			elevatorS = new ElevatorS(new ElevatorIO(){});
			doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIO(){});
		}
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		List<Pair<String, Command>> autoCommands = Arrays.asList(
				//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
				new Pair<String, Command>("BranchGrabbingGamePiece",
						new BranchAuto("Shoot",
								new Pose2d(7.4, 5.8, new Rotation2d()), 4)),
				//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				new Pair<String, Command>("SimBot", new SimDefenseBot()));
		Pathfinding.setPathfinder(new LocalADStarAK());
		NamedCommands.registerCommands(autoCommands);
		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}
		PathfindingCommand.warmupCommand()
				.andThen(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						DriveConstants.pathConstraints, drivetrainS, false, 0))
				.finallyDo(() -> RobotContainer.field.getObject("target pose")
						.setPose(new Pose2d(-50, -50, new Rotation2d())))
				.schedule();
		flywheelS.setDefaultCommand(new FlywheelC(flywheelS));
		armS.setDefaultCommand(new SingleJointedArmC(armS));
		elevatorS.setDefaultCommand(new ElevatorC(elevatorS));
		doubleJointedArmS.setDefaultCommand(new DoubleJointedArmC(doubleJointedArmS));
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(auto -> {
			try {
				field.getObject("path")
						.setPoses(PathFinder.parseAutoToPose2dList(auto.getName()));
			}
			catch (Exception e) {
				System.err.println("NO FOUND PATH FOR DESIRED AUTO!!");
				field.getObject("path").setPoses(
						new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
								new Pose2d(-50.2, -50, new Rotation2d())
				});
			}
		});
		// Configure the trigger bindings
		configureBindings();
		addNTCommands();
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		// Some condition that should decide if we want to override rotation
		return angleOverrider;
	}

	private void configureBindings() {
		xButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.onTrue(new InstantCommand(() -> drivetrainS.zeroHeading()));
		yButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, true, drivetrainS));
		bButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, true, drivetrainS));
		aButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, false, drivetrainS));
		xButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, false, drivetrainS));
		//Example Drive To 2024 Amp Pose, Bind to what you need.
		/*yButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						DriveConstants.pathConstraints, drivetrainS, false, 0));*/
		//swerve DRIVE tests
		//When user hits right bumper, go to next test, or wrap back to starting test for SysID.
		rightBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == Constants.SysIdRoutines.values().length - 1) {
				currentTest = 0;
				System.out.println("looping");
			} else {
				currentTest++;
			}
		}));
		//When user hits left bumper, go to next test, or wrap back to starting test for SysID.
		leftBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == 0) {
				currentTest = Constants.SysIdRoutines.values().length - 1;
				System.out.println("looping");
			} else {
				currentTest--;
			}
		}));
		//When using CTRE, be sure to hit Start so that the motors are logged via CTRE (For SysId)
		selectButtonTest.onTrue(Commands.runOnce(SignalLogger::stop));
		startButtonTest.onTrue(Commands.runOnce(SignalLogger::start));
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

	/**
	 * For SIMULATION ONLY, return the estimated current draw of the robot.
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {
		return new double[] { Math.min(drivetrainS.getCurrent(), 200),
      flywheelS.getCurrent(),
			armS.getCurrent(),
			elevatorS.getCurrent()
		};
	}

	private static void addNTCommands() {
		SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
	}

	/**
	 * RUN EACH system's test command. Does NOT run any checks on vision.
	 * 
	 * @return a command with all of them in a sequence.
	 */
	public static Command allSystemsCheck() {
	return Commands.sequence(drivetrainS.getRunnableSystemCheckCommand(),flywheelS.getSystemCheckCommand(),armS.getSystemCheckCommand(),elevatorS.getSystemCheckCommand(),doubleJointedArmS.getSystemCheckCommand());
	}

	public static HashMap<String, Double> combineMaps(
			List<HashMap<String, Double>> maps) {
		HashMap<String, Double> combinedMap = new HashMap<>();
		// Iterate over the list of maps
		for (HashMap<String, Double> map : maps) {
			combinedMap.putAll(map);
		}
		return combinedMap;
	}

	public static HashMap<String, Double> getAllTemps() {
		// List of HashMaps
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(),
		flywheelS.getTemps(),
		armS.getTemps(),
		elevatorS.getTemps(),
		doubleJointedArmS.getTemps());

		// Combine all maps
		HashMap<String, Double> combinedMap = combineMaps(maps);
		return combinedMap;
	}

	/**
	 * Checks EACH system's status (DOES NOT RUN THE TESTS)
	 * 
	 * @return true if ALL systems were good.
	 */
	public static boolean allSystemsOK() {
		return drivetrainS.getTrueSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& flywheelS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& elevatorS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& armS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& doubleJointedArmS.getSystemStatus() == SubsystemChecker.SystemStatus.OK;
	 }
	public static Collection<ParentDevice> getOrchestraDevices() {
    
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		devices.addAll(flywheelS.getOrchestraDevices());
		devices.addAll(elevatorS.getOrchestraDevices());
    	devices.addAll(armS.getOrchestraDevices());
		devices.addAll(doubleJointedArmS.getOrchestraDevices());

    return devices;
	}
	public static Subsystem[] getAllSubsystems(){
		Subsystem[] subsystems = new Subsystem[5];
		subsystems[0] = drivetrainS;
    	subsystems[1] = flywheelS;
    	subsystems[2] = elevatorS;
    	subsystems[3] = armS;
    	subsystems[4] = doubleJointedArmS;
		return subsystems;
	}
}
