package frc.robot.Subsystems.Indexer;

import static frc.robot.Subsystems.Indexer.IndexerConstants.*;

import org.team7525.subsystem.SubsystemStates;

public enum IndexerState implements SubsystemStates {
	INTAKING("Intaking", -INTAKING_SPEED),
	IDLE("idle", IDLE_SPEED),
	SHOOTING("SHooting", SHOOTING_SPEED);

	private String stateString;
	private double motorSpeed;

	IndexerState(String stateString, double motorSpeed) {
		this.stateString = stateString;
		this.motorSpeed = motorSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getMotorSpeed() {
		return motorSpeed;
	}
}
