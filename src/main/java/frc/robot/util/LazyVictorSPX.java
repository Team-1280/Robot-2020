package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Sends only new commands to the Talon to reduce CAN usage.
 *
 */
public class LazyVictorSPX extends VictorSPX {

	private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	public LazyVictorSPX(int deviceNumber) {
		super(deviceNumber);
		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

}
