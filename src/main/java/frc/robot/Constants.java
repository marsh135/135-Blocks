// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**
 * Anything for the e n t i r e robot goes here. Specific constants go under
 * Utils, and their respective folder for Merge purposes.
 */
public final class Constants {
	public static final Mode currentMode = Mode.SIM;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,
		/** Running a physics simulator. */
		SIM,
		/** Replaying from a log file. */
		REPLAY
	}

	//put datalog constants IN THE UTIL FOR THAT FILE. 
	public static Map<Integer, String> manCanIdsToNames() {
		HashMap<Integer, String> map = new HashMap<>();
		//this has to be adjusted for every block branch added!
		//map.put(Constants.OutakeConstants.topFlywheel, "topFlywheel");
		return map;
	}
}