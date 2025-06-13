package frc.robot.Manager;

import frc.robot.Subsystems.AdjustableHood.AdjustableHoodStates;
import frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemStates;
import frc.robot.Subsystems.Indexer.IndexerState;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.ShooterStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"Idle",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		AdjustableHoodStates.IDLE,
		HoodedShooterSupersystemStates.IDLE,
		ShooterStates.IDLE
	),
	OUTTAKING(
		"Outtaking",
		IntakeStates.OUTTAKING,
		IndexerState.IDLE,
		AdjustableHoodStates.IDLE,
		HoodedShooterSupersystemStates.IDLE,
		ShooterStates.IDLE
	),
	INTAKING(
		"Intaking",
		IntakeStates.INTAKING,
		IndexerState.IDLE,
		AdjustableHoodStates.IDLE,
		HoodedShooterSupersystemStates.IDLE,
		ShooterStates.IDLE
	),
	INTAKE_PASSING(
		"Passing",
		IntakeStates.PASSING,
		IndexerState.INTAKING,
		AdjustableHoodStates.IDLE,
		HoodedShooterSupersystemStates.IDLE,
		ShooterStates.IDLE
	),
	FIXED_ALIGN(
		"Fixed Align",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		AdjustableHoodStates.FIXED,
		HoodedShooterSupersystemStates.FIXED,
		ShooterStates.FIXED
	),
	DYNAMIC_ALIGN(
		"Dynamic Align",
		IntakeStates.IDLE,
		IndexerState.IDLE,
		AdjustableHoodStates.DYNAMIC,
		HoodedShooterSupersystemStates.DYNAMIC,
		ShooterStates.DYNAMIC
	),
	DYNAMIC_SHOOT(
		"Dynamic Shoot",
		IntakeStates.IDLE,
		IndexerState.SHOOTING,
		AdjustableHoodStates.DYNAMIC,
		HoodedShooterSupersystemStates.DYNAMIC,
		ShooterStates.DYNAMIC
	),
	FIXED_SHOOT(
		"Fixed Shoot",
		IntakeStates.IDLE,
		IndexerState.SHOOTING,
		AdjustableHoodStates.FIXED,
		HoodedShooterSupersystemStates.FIXED,
		ShooterStates.FIXED
	);

	private String stateString;
	private IntakeStates intake;
	// private HoodAndShooterStates hoodAndShooter;
	private IndexerState indexer;
	private AdjustableHoodStates adjustableHood;
	private HoodedShooterSupersystemStates hoodedShooterSupersystem;
	private ShooterStates shooter;

	ManagerStates(
		String stateString,
		IntakeStates intake,
		IndexerState indexer,
		AdjustableHoodStates adjustableHood,
		HoodedShooterSupersystemStates hoodedShooterSupersystem,
		ShooterStates shooter
	) {
		this.stateString = stateString;
		this.intake = intake;
		this.indexer = indexer;
		this.adjustableHood = adjustableHood;
		this.hoodedShooterSupersystem = hoodedShooterSupersystem;
		this.shooter = shooter;
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

	public AdjustableHoodStates getAdjustableHoodStates() {
		return adjustableHood;
	}

	public HoodedShooterSupersystemStates getHoodedShooterSupersystemStates() {
		return hoodedShooterSupersystem;
	}

	public ShooterStates getShooterStates() {
		return shooter;
	}
}
