package frc.robot.Subsystems.AdjustableHood;

import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.*;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystem;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum AdjustableHoodStates implements SubsystemStates {
	IDLE("IDLE", () -> IDLE_ANGLE),
	FIXED("FIXED", () -> FIXED_ANGLE),
	DYNAMIC("DYNAMIC", () -> HoodedShooterSupersystem.getInstance().calculateHoodSetpoint());

	private final String stateName;
	private final Supplier<Angle> setpointSupplier;

	AdjustableHoodStates(String stateName, Supplier<Angle> setpointSupplier) {
		this.stateName = stateName;
		this.setpointSupplier = setpointSupplier;
	}

	@Override
	public String getStateString() {
		return stateName;
	}

	public Angle getSetpoint() {
		return setpointSupplier.get();
	}
}
