package frc.robot.Subsystems.Drive;

import frc.robot.TeamLib.subsystem.*;

public enum DriveStates implements SubsystemStates {
	ARCADE_DRIVE("Arcade Drive", () -> {
        Drive drive = Drive.getInstance();
        double forward = drive.getController().getLeftY();
        double rotation = drive.getController().getRightX();
        if (drive.isSlowMode()) {
            forward *= DriveConstants.SLOW_MODE_MULTIPLIER;
            rotation *= DriveConstants.SLOW_MODE_MULTIPLIER;
        }
        drive.arcadeDrive(forward, rotation);
	}),
	TANK_DRIVE("Tank Drive", () -> {
        Drive drive = Drive.getInstance();
        double left = drive.getController().getLeftY();
        double right = drive.getController().getRightY();
        if (drive.isSlowMode()) {
            left *= DriveConstants.SLOW_MODE_MULTIPLIER;
            right *= DriveConstants.SLOW_MODE_MULTIPLIER;
        }
        drive.tankDrive(left, right);
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
