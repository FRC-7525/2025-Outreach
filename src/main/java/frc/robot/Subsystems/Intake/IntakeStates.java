package frc.robot.Subsystems.Intake;

import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
	INTAKING(
		"INTAKING",
		IntakeConstants.INTAKING_SPEED.magnitude(),
		IntakeConstants.INTAKING_PIVOT.magnitude()
	),
	OUTTAKING(
		"OUTTAKING",
		IntakeConstants.OUTTAKING_SPEED.magnitude(),
		IntakeConstants.OUTTAKING_PIVOT.magnitude()
	),
	PASSING(
		"PASSING",
		IntakeConstants.PASSING_SPEED.magnitude(),
		IntakeConstants.PASSING_PIVOT.magnitude()
	),
	IDLE("IDLE", IntakeConstants.IDLE_SPEED.magnitude(), IntakeConstants.IDLE_PIVOT.magnitude());

	IntakeStates(String stateString, Double wheelSpeed, double pivotSetpoint) {
		this.wheelSpeed = wheelSpeed;
		this.pivotSetpoint = pivotSetpoint;
		this.stateString = stateString;
	}

	private double wheelSpeed;
	private double pivotSetpoint;
	private String stateString;

	public double getWheelSpeedSetpoint() {
		return wheelSpeed;
	}

	public double getPivotSetpoint() {
		return pivotSetpoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
