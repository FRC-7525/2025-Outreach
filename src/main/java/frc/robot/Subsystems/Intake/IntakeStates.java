package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
	INTAKING(
		"INTAKING",
		IntakeConstants.INTAKING_SPEED.in(RotationsPerSecond),
		IntakeConstants.INTAKING_PIVOT.in(Degrees)
	),
	OUTTAKING(
		"OUTTAKING",
		IntakeConstants.OUTTAKING_SPEED.in(RotationsPerSecond),
		IntakeConstants.OUTTAKING_PIVOT.in(Degrees)
	),
	PASSING(
		"PASSING",
		IntakeConstants.PASSING_SPEED.in(RotationsPerSecond),
		IntakeConstants.PASSING_PIVOT.in(Degrees)
	),
	IDLE("IDLE", 
		IntakeConstants.IDLE_SPEED.in(RotationsPerSecond), 
		IntakeConstants.IDLE_PIVOT.in(Degrees)
	);

	IntakeStates(String stateString, double wheelSpeed, double pivotSetpoint) {
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
