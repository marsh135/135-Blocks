package frc.robot.subsystems.CTRE_state_space;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.Constants;
import frc.robot.DataHandler;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;
import java.util.ArrayList;
import java.util.HashMap;
public class CTREDoubleJointedArmS extends SubsystemChecker {
	private TalonFX armMotor = new TalonFX(
			CTRESpaceConstants.DoubleJointedArm.kArmMotorID);
	private TalonFX elbowMotor = new TalonFX(
			CTRESpaceConstants.DoubleJointedArm.kElbowMotorID);
	private Notifier m_updatePositionsNotifier = null; //Checks for updates
	public static double expectedArmRads = 0, expectedElbowRads = 0;
	private final VoltageOut m_voltReq = new VoltageOut(0.0);
	private final Mechanism2d m_mech2d = new Mechanism2d(
			CTRESpaceConstants.DoubleJointedArm.simSizeWidth,
			CTRESpaceConstants.DoubleJointedArm.simSizeLength);
	private final MechanismRoot2d m_DoubleJointedarmPivot = m_mech2d.getRoot(
			"DoubleJointedArmPivot", CTRESpaceConstants.DoubleJointedArm.physicalX,
			CTRESpaceConstants.DoubleJointedArm.physicalY);
	private final MechanismLigament2d m_DoubleJointedArm = m_DoubleJointedarmPivot
			.append(new MechanismLigament2d("DoubleJointedArm",
					CTRESpaceConstants.DoubleJointedArm.armLength,
					Units.radiansToDegrees(expectedArmRads), 1,
					new Color8Bit(Color.kYellow)));
	private final MechanismLigament2d m_DoubleJointedElbow = m_DoubleJointedArm
			.append(new MechanismLigament2d("DoubleJointedElbow",
					CTRESpaceConstants.DoubleJointedArm.elbowLength,
					Units.radiansToDegrees(expectedElbowRads), 1,
					new Color8Bit(Color.kYellow)));
  private final StatusSignal<Double> m_armpos = armMotor.getPosition();
  private final StatusSignal<Double> m_elbowpos = elbowMotor.getPosition();
  private final StatusSignal<Double> m_armvel = armMotor.getVelocity();
  private final StatusSignal<Double> m_elbowvel = elbowMotor.getVelocity();
	public CTREDoubleJointedArmS() {
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.DoubleJointedArm.armStatorCurrentLimit;
		motorConfig.MotorOutput.Inverted = CTRESpaceConstants.DoubleJointedArm.inverted;
		motorConfig.MotorOutput.NeutralMode = CTRESpaceConstants.DoubleJointedArm.mode;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.DoubleJointedArm.armGearing;
		armMotor.getConfigurator().apply(motorConfig);
		//do elbow
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.DoubleJointedArm.elbowStatorCurrentLimit;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.DoubleJointedArm.elbowGearing;
		elbowMotor.getConfigurator().apply(motorConfig);
		if (Constants.currentMode == Constants.Mode.SIM){
			m_updatePositionsNotifier = new Notifier(() -> {
				DataHandler.logData(new double[]{expectedArmRads,expectedElbowRads},"DoubleJointedEncoders");
			});
		}else{
			m_updatePositionsNotifier = new Notifier(() -> {
				DataHandler.logData(getEncoders(),"DoubleJointedEncoders");
			});
		}

		m_updatePositionsNotifier.startPeriodic(1);
		registerSelfCheckHardware();
	}

	public void setMotors(double voltsArm, double voltsElbow) {
		armMotor.setControl(m_voltReq.withOutput(voltsArm));
		elbowMotor.setControl(m_voltReq.withOutput(voltsElbow));
	}

	public double[] getEncoders() {
		BaseStatusSignal.refreshAll(m_armpos, m_armvel,m_elbowpos, m_elbowvel);
		return new double[] {
			Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(m_armpos, m_armvel)),
			Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(m_elbowpos, m_elbowvel))
		};
	}
	public double getArmError(double armSet){
		if (Constants.currentMode == Constants.Mode.SIM){
			return expectedArmRads-armSet;
		}
		return getEncoders()[0]-armSet;
	}
	public double getElbowError(double elbowSet){
		if (Constants.currentMode == Constants.Mode.SIM){
			return expectedElbowRads-elbowSet;
		}
		return getEncoders()[1]-elbowSet;
	}
	@Override
	public void periodic() {
		if (Constants.currentMode == Constants.Mode.REAL) {
			m_DoubleJointedArm.setAngle(Units
					.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()));
			m_DoubleJointedElbow.setAngle(Units
					.rotationsToDegrees(elbowMotor.getPosition().getValueAsDouble()));
		} else {
			m_DoubleJointedArm.setAngle(Units.radiansToDegrees(expectedArmRads));
			m_DoubleJointedElbow
					.setAngle(Units.radiansToDegrees(expectedElbowRads));
		}
		Logger.recordOutput("DoubleJointedArmMechanism", m_mech2d);
	}
	public void registerSelfCheckHardware() {
		super.registerHardware("DoubleArmMotor", armMotor);
		super.registerHardware("DoubleElbowMotor", elbowMotor);

	  }
	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>(2);
		orchestra.add(armMotor);
		orchestra.add(elbowMotor);
		return orchestra;
	 }
	 public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("DoubleArmMotorTemp", armMotor.getDeviceTemp().getValueAsDouble());
		tempMap.put("DoubleElbowMotorTemp", elbowMotor.getDeviceTemp().getValueAsDouble());
		return tempMap;
	}
	public double getArmRads(){
		if (Constants.currentMode == Constants.Mode.SIM){
			return expectedArmRads;
		}
		return getEncoders()[0];
	}
	public double getElbowRads(){
		if (Constants.currentMode == Constants.Mode.SIM){
			return expectedElbowRads;
		}
		return getEncoders()[1];
	}
	/**
	 * @return a double list which contains an x coordinate in meters for the endpoint, and y.
	 */
	public double[] getCoordinate(){
		double armPos = getArmRads();
		double elbowPos = getElbowRads()+getArmRads();
		double x = Math.cos(armPos) * CTRESpaceConstants.DoubleJointedArm.armLength + Math.cos(elbowPos) * CTRESpaceConstants.DoubleJointedArm.elbowLength;
		double y = Math.sin(armPos) * CTRESpaceConstants.DoubleJointedArm.armLength + Math.sin(elbowPos) * CTRESpaceConstants.DoubleJointedArm.elbowLength;
		return new double[]{x,y};
	}
	@Override
	protected Command systemCheckCommand() { 
	return Commands.sequence(run(() -> DataHandler.logData(
		CTRESpaceConstants.DoubleJointedArm.macroTopRight,
		"DoubleJointSetpoint")).withTimeout(5),
				runOnce(() ->{
					double[] coordinate = getCoordinate();
					if (Math.abs(coordinate[0]-CTRESpaceConstants.DoubleJointedArm.macroTopRight[0]) > Units.inchesToMeters(5) || Math.abs(coordinate[1]-CTRESpaceConstants.DoubleJointedArm.macroTopRight[1]) > Units.inchesToMeters(5)){
						addFault("[System Check] Arm position was off more than 5 in. Wanted 1.5,1.1, got " + coordinate[0] + "," + coordinate[1], false,true);
					}
				}),run(() -> DataHandler.logData(
					CTRESpaceConstants.DoubleJointedArm.macroTopLeft,
					"DoubleJointSetpoint")).withTimeout(5),
							runOnce(() ->{
								double[] coordinate = getCoordinate();
								if (Math.abs(coordinate[0]-CTRESpaceConstants.DoubleJointedArm.macroTopLeft[0]) > Units.inchesToMeters(5) || Math.abs(coordinate[1]-CTRESpaceConstants.DoubleJointedArm.macroTopLeft[1]) > Units.inchesToMeters(5)){
									addFault("[System Check] Arm position was off more than 5 in. Wanted -1.5,1.0, got " + coordinate[0] + "," + coordinate[1], false,true);
								}
							}))
        .until(
            () ->
                !getFaults().isEmpty());
	}
}
