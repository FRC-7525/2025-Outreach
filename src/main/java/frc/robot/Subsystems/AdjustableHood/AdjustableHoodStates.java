package frc.robot.Subsystems.AdjustableHood;

import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.*;

import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum AdjustableHoodStates implements SubsystemStates {
	IDLE("IDLE", IDLE_ANGLE),
	FIXED("FIXED", FIXED_ANGLE),
	DYNAMIC("DYNAMIC", null);

	private final String stateName;
	private final Angle setpoint;

	AdjustableHoodStates(String stateName, Angle setpoint) {
		this.stateName = stateName;
		this.setpoint = setpoint;
	}

	public String getStateString() {
		return stateName;
	}

	public Angle getSetpoint() {
		return setpoint;
	}
}
