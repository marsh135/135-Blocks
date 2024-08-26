package frc.robot.subsystems.leds;

import frc.robot.utils.leds.LEDConstants.LEDStates;

public class LEDState{
	public LEDStates state;
	public int panelIndex;
	public LEDState(LEDStates state, int panelIndex){
		this.state = state;
		this.panelIndex = panelIndex;
	}
}
