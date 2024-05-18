package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.Robot;
import frc.robot.utils.leds.LEDConstants;

public class LEDs extends SubsystemBase {
	public static AddressableLED leds;
	public static AddressableLEDBuffer ledBuffer;
	public static AddressableLEDSim ledSim;

	public LEDs() {
		//creates LED objects (the actual LEDs, and a buffer that stores data to be sent to them)
		leds = new AddressableLED(LEDConstants.ledPort);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
		//sets length of the LED strip to buffer length
		leds.setLength(ledBuffer.getLength());
		//starts LED strips
		leds.start(); //FOR THE LOVE OF GOD PLEASE REMEMBER THIS IF YOU'RE GONNA CODE YOUR OWN SUBSYSTEM I SPENT LIKE 6 HOURS TROUBLESHOOTING AND IT DIDNT WORK BECAUSE OF THIS
		//if the robot is a simulation, create a simulation for the addressableLEDs
		if (Robot.isSimulation()) {
			ledSim = new AddressableLEDSim(leds);
		}
	}
}
