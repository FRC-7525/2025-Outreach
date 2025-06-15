package frc.robot.Subsystems.Drive;

import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	FIELD_RELATIVE("Field Relative", () -> {
		Drive.getInstance().getDrive().driveFieldOriented(Drive.getInstance().getSwerveInputs());
	}),
	ROBOT_RELATIVE("Robot Relative", () -> {
		Drive.getInstance().getDrive().drive(Drive.getInstance().getSwerveInputs());
	});

	private String stateString;
	private Runnable driveControl;

	DriveStates(String stateString, Runnable driveControl) {
		this.stateString = stateString;
		this.driveControl = driveControl;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public void driveRobot() {
		driveControl.run();
	}
}
