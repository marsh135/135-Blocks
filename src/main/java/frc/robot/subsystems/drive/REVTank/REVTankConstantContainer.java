package frc.robot.subsystems.drive.REVTank;

import com.revrobotics.CANSparkBase.IdleMode;

public class REVTankConstantContainer {
	private IdleMode idleMode;
	private int[] REVTankInts;
	private double[] REVTankDoubles;
	private boolean[] REVTankBooleans;

	public REVTankConstantContainer(int leftMasterID, int leftFollowerID,
			int rightMasterID, int rightFollowerID, boolean leftMasterInverted,
			boolean leftFollowerInverted, boolean rightMasterInverted,
			boolean rightFollowerInverted, IdleMode idleMode, int maxAmps,
			double gearing, double kWheelDiameterMeters, double kChassisLength) {
		REVTankInts = new int[] { leftMasterID, leftFollowerID, rightMasterID,
				rightFollowerID, maxAmps
		};
		REVTankBooleans = new boolean[] { leftMasterInverted,
				leftFollowerInverted, rightMasterInverted, rightFollowerInverted
		};
		REVTankDoubles = new double[] { gearing, kWheelDiameterMeters,
				kChassisLength
		};
		this.idleMode = idleMode;
	}

	public int getLeftMasterID() { return REVTankInts[0]; }

	public int getRightMasterID() { return REVTankInts[1]; }

	public int getLeftFollowerID() { return REVTankInts[2]; }

	public int getRightFollowerID() { return REVTankInts[3]; }

	public int getMaxAmps() { return REVTankInts[4]; }

	public boolean getLeftMasterInverted() { return REVTankBooleans[0]; }

	public boolean getLeftFollowerInverted() { return REVTankBooleans[1]; }

	public boolean getRightMasterInverted() { return REVTankBooleans[2]; }

	public boolean getRightFollowerInverted() { return REVTankBooleans[3]; }

	public double getGearing() { return REVTankDoubles[0]; }

	public double getWheelDiameterMeters() { return REVTankDoubles[1]; }

	public double getChassisLength() { return REVTankDoubles[2]; }

	public IdleMode getIdleMode(){
		return idleMode;
	}
}
