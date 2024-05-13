# 135 Blocks
This is designed to be a template-based way for FRC programming.

This is designed so that the only files that need changing to combine different subsystems, if done properly, is RobotContainer.java.
#### Every single block has the following subsystems built in for ease of development:

- Swerve Drive
- Custom Data Handler
- AdvantageKit

#### Preferably, do NOT touch any of the drive/* contents, as that could cause a merge error!
###### However, some files do need to change interactions with the base, so it is understandable in some situations.
#### An example branch directory could be

    src/main/java/frc/robot/
        commands/
		drive/
			...
        	exampleBranchName/
			exampleCommand.java
        subsystems/
		drive/
			...
		exampleBranchName/
			exampleSubsystem.java
        utils/
		drive/
			...
		exampleBranchName/
			Constants.java
        Constants.java
	DataHandler.java
	Main.java
	Robot.java
	RobotContainer.java
