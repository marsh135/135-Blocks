package frc.robot.subsystems.state_space.DoubleJointedArm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DataHandler;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.state_space.StateSpaceConstants;

public class DoubleJointedArmS extends SubsystemChecker {
	private final DoubleJointedArmIO io;
	private final DoubleJointedArmIOInputsAutoLogged inputs = new DoubleJointedArmIOInputsAutoLogged();
	private List<Double> voltages;
	private double armSetRad;
	private double elbowSetRad;
	public double latency;
	private Notifier m_updatePositionsNotifier = null; //Checks for updates
	private final Mechanism2d m_mech2d = new Mechanism2d(
			StateSpaceConstants.DoubleJointedArm.simSizeWidth,
			StateSpaceConstants.DoubleJointedArm.simSizeLength);
	private final MechanismRoot2d m_DoubleJointedarmPivot = m_mech2d.getRoot(
			"DoubleJointedArmPivot",
			StateSpaceConstants.DoubleJointedArm.physicalX,
			StateSpaceConstants.DoubleJointedArm.physicalY);
	private final MechanismLigament2d m_DoubleJointedArm = m_DoubleJointedarmPivot
			.append(new MechanismLigament2d("DoubleJointedArm",
					StateSpaceConstants.DoubleJointedArm.armLength,
					Units.radiansToDegrees(getArmRads()), 1,
					new Color8Bit(Color.kYellow)));
	private final MechanismLigament2d m_DoubleJointedElbow = m_DoubleJointedArm
			.append(new MechanismLigament2d("DoubleJointedElbow",
					StateSpaceConstants.DoubleJointedArm.elbowLength,
					Units.radiansToDegrees(getElbowRads()), 1,
					new Color8Bit(Color.kYellow)));

	public DoubleJointedArmS(DoubleJointedArmIO io) {
		this.io = io;
		m_updatePositionsNotifier = new Notifier(() -> {
			DataHandler.logData(new double[] { getArmRads(), getElbowRads()
			},"DoubleJointedEncoders");
		});
		m_updatePositionsNotifier.startPeriodic(.02);
		registerSelfCheckHardware();
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(io.getSelfCheckingHardware());
	}

	public void setVoltages(List<Double> voltages) { this.voltages = voltages; }

	@Override
	public void periodic() {
		if (voltages != null) {
			io.setVoltage(voltages);
		}
		io.updateInputs(inputs);
		Logger.processInputs("DoubleJointedArmS", inputs);
		m_DoubleJointedArm.setAngle(Units.radiansToDegrees(getArmRads()));
		m_DoubleJointedElbow.setAngle(Units.radiansToDegrees(getElbowRads()));
		Logger.recordOutput("DoubleJointedArmMechanism", m_mech2d);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = io.getSelfCheckingHardware();
		for (SelfChecking motor : driveHardware) {
			if (motor.getHardware() instanceof TalonFX) {
				orchestra.add((TalonFX) motor.getHardware());
			}
		}
		return orchestra;
	}

	public double getArmError() { return inputs.positionArmRads - armSetRad; }
	public void setExpectedPositions(List<Double> rads){ io.setExpectedPositions(rads.get(0), rads.get(1));}
	public double getElbowError() {
		return inputs.positionElbowRads - elbowSetRad;
	}
	public void setDoubleJointedArm(double armRad, double elbowRad, double elbowInversed){
		this.armSetRad = armRad;
		this.elbowSetRad = elbowRad;
		DataHandler.logData(new double[]{armRad,elbowRad,elbowInversed},"DoubleJointSetpoint");
	}
	public void setDoubleJointedArm(double[] macro){
		this.armSetRad = macro[0];
		this.elbowSetRad = macro[1];
		DataHandler.logData(macro,"DoubleJointSetpoint");
	}
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("DoubleArmMotorTemp", inputs.armTemp);
		tempMap.put("DoubleElbowMotorTemp", inputs.elbowTemp);
		return tempMap;
	}

	public double getArmRads() { return inputs.positionArmRads; }

	public double getElbowRads() { return inputs.positionElbowRads; }

	/**
	 * @return a double list which contains an x coordinate in meters for the
	 *         endpoint, and y.
	 */
	public double[] getCoordinate() {
		double armPos = getArmRads();
		double elbowPos = getElbowRads() + getArmRads();
		double x = Math.cos(armPos)
				* StateSpaceConstants.DoubleJointedArm.armLength
				+ Math.cos(elbowPos)
						* StateSpaceConstants.DoubleJointedArm.elbowLength;
		double y = Math.sin(armPos)
				* StateSpaceConstants.DoubleJointedArm.armLength
				+ Math.sin(elbowPos)
						* StateSpaceConstants.DoubleJointedArm.elbowLength;
		return new double[] { x, y
		};
	}

	@Override
	public double getCurrent() {
		return Math.abs(inputs.currentAmps[0]) + Math.abs(inputs.currentAmps[1]);
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(run(() -> DataHandler.logData(
				StateSpaceConstants.DoubleJointedArm.macroTopRight,
				"DoubleJointSetpoint")).withTimeout(5), runOnce(() -> {
					double[] coordinate = getCoordinate();
					if (Math.abs(coordinate[0]
							- StateSpaceConstants.DoubleJointedArm.macroTopRight[0]) > Units
									.inchesToMeters(5)
							|| Math.abs(coordinate[1]
									- StateSpaceConstants.DoubleJointedArm.macroTopRight[1]) > Units
											.inchesToMeters(5)) {
						addFault(
								"[System Check] Arm position was off more than 5 in. Wanted 1.5,1.1, got "
										+ coordinate[0] + "," + coordinate[1],
								false, true);
					}
				}),
				run(() -> DataHandler.logData(
						StateSpaceConstants.DoubleJointedArm.macroTopLeft,
						"DoubleJointSetpoint")).withTimeout(5),
				runOnce(() -> {
					double[] coordinate = getCoordinate();
					if (Math.abs(coordinate[0]
							- StateSpaceConstants.DoubleJointedArm.macroTopLeft[0]) > Units
									.inchesToMeters(5)
							|| Math.abs(coordinate[1]
									- StateSpaceConstants.DoubleJointedArm.macroTopLeft[1]) > Units
											.inchesToMeters(5)) {
						addFault(
								"[System Check] Arm position was off more than 5 in. Wanted -1.5,1.0, got "
										+ coordinate[0] + "," + coordinate[1],
								false, true);
					}
				})).until(() -> !getFaults().isEmpty());
	}
}
