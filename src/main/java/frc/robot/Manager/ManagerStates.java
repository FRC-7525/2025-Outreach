package frc.robot.Manager;

import frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemStates;
import frc.robot.Subsystems.Indexer.IndexerState;
import frc.robot.Subsystems.Intake.IntakeStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"Idle",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		HoodedShooterSupersystemStates.IDLE
	),
	OUTTAKING(
		"Outtaking",
		IntakeStates.OUTTAKING,
		IndexerState.IDLE,
		HoodedShooterSupersystemStates.IDLE
	),
	INTAKING(
		"Intaking",
		IntakeStates.INTAKING,
		IndexerState.IDLE,
		HoodedShooterSupersystemStates.IDLE
	),
	INTAKE_PASSING(
		"Passing",
		IntakeStates.PASSING,
		IndexerState.INTAKING,
		HoodedShooterSupersystemStates.IDLE
	),
	FIXED_ALIGN(
		"Fixed Align",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		HoodedShooterSupersystemStates.FIXED
	),
	DYNAMIC_ALIGN(
		"Dynamic Align",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		HoodedShooterSupersystemStates.DYNAMIC
	),
	DYNAMIC_SHOOT(
		"Dynamic Shoot",
		IntakeStates.IDLE,
		IndexerState.SHOOTING,
		HoodedShooterSupersystemStates.DYNAMIC
	),
	FIXED_SHOOT(
		"Fixed Shoot",
		IntakeStates.IDLE,
		IndexerState.SHOOTING,
		HoodedShooterSupersystemStates.FIXED
	);

	private String stateString;
	private IntakeStates intake;
	private IndexerState indexer;
	private HoodedShooterSupersystemStates hoodedShooterSupersystem;

	ManagerStates(
		String stateString,
		IntakeStates intake,
		IndexerState indexer,
		HoodedShooterSupersystemStates hoodedShooterSupersystem
	) {
		this.stateString = stateString;
		this.intake = intake;
		this.indexer = indexer;
		this.hoodedShooterSupersystem = hoodedShooterSupersystem;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public IntakeStates getIntakeStates() {
		return intake;
	}

	public IndexerState getIndexerStates() {
		return indexer;
	}

	public HoodedShooterSupersystemStates getHoodedShooterSupersystemStates() {
		return hoodedShooterSupersystem;
	}
}
