package frc.robot.Subsystems.Indexer;

import org.team7525.subsystem.SubsystemStates;

public enum IndexerState implements SubsystemStates {
	INTAKING("Intaking", -0.5),
	IDLE("idle", 0.0),
	SHOOTING("SHooting", 1.0);

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
