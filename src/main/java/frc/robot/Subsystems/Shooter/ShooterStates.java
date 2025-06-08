package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

public enum ShooterStates implements SubsystemStates {
	IDLE("IDLE", RotationsPerSecond.of(0)),
	FIXED("FIXED", FIXED_SPEED),
	DYNAMIC("DYNAMIC", null);

	private final String stateName;
	private final AngularVelocity setpoint;

	ShooterStates(String stateName, AngularVelocity setpoint) {
		this.stateName = stateName;
		this.setpoint = setpoint;
	}

	public String getStateString() {
		return stateName;
	}

	public AngularVelocity getSetpoint() {
		return setpoint;
	}
}
