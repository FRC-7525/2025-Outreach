package frc.robot.Subsystems.HoodedShooterSupersystem;

import frc.robot.Subsystems.AdjustableHood.AdjustableHoodStates;
import frc.robot.Subsystems.Shooter.ShooterStates;
import org.team7525.subsystem.SubsystemStates;

public enum HoodedShooterSupersystemStates implements SubsystemStates {
	IDLE("IDLE", AdjustableHoodStates.IDLE, ShooterStates.IDLE),
	FIXED("FIXED", AdjustableHoodStates.FIXED, ShooterStates.FIXED),
	DYNAMIC("DYNAMIC", AdjustableHoodStates.DYNAMIC, ShooterStates.DYNAMIC);

	private final String stateString;
	private final AdjustableHoodStates adjustableHoodState;
	private final ShooterStates shooterState;

	HoodedShooterSupersystemStates(
		String stateString,
		AdjustableHoodStates adjustableHoodState,
		ShooterStates shooterState
	) {
		this.stateString = stateString;
		this.adjustableHoodState = adjustableHoodState;
		this.shooterState = shooterState;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AdjustableHoodStates getAdjustableHoodState() {
		return adjustableHoodState;
	}

	public ShooterStates getShooterState() {
		return shooterState;
	}
}
