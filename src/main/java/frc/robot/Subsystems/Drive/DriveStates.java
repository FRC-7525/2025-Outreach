package frc.robot.Subsystems.Drive;

import frc.robot.TeamLib.subsystem.*;

public enum DriveStates implements SubsystemStates {
	ARCADE_DRIVE("Arcade Drive", () -> {
        Drive.getInstance().arcadeDrive(
			Drive.getInstance().getController().getLeftY(),
			Drive.getInstance().getController().getRightX()
		);
	}),
	TANK_DRIVE("Tank Drive", () -> {
		Drive.getInstance().tankDrive(
			Drive.getInstance().getController().getLeftY(),
			Drive.getInstance().getController().getRightY()
		);
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
