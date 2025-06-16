package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;
import java.util.function.Supplier;

public enum ShooterStates implements SubsystemStates {
	IDLE("IDLE", () -> RotationsPerSecond.of(ZERO)),
	FIXED("FIXED", () -> FIXED_SPEED),
	DYNAMIC("DYNAMIC", ShooterStates::getDynamicSetpoint);

	private final String stateName;
	private final Supplier<AngularVelocity> setpointSupplier;

	ShooterStates(String stateName, Supplier<AngularVelocity> setpointSupplier) {
		this.stateName = stateName;
		this.setpointSupplier = setpointSupplier;
	}

	public String getStateString() {
		return stateName;
	}

	public AngularVelocity getSetpoint() {
		return setpointSupplier.get();
	}

	// Example dynamic setpoint method. Replace with your actual logic.
	private static AngularVelocity getDynamicSetpoint() {
		return RotationsPerSecond.of(0); // Placeholder
	}
}
