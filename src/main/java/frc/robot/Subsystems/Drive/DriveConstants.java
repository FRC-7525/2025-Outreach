package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {

	//Btw im just using frontier's numbers
	public static final String SUBSYSTEM_NAME = "Drive";

    //Init Pose
    public static final double INIT_POSE_X = 9.9;
    public static final double INIT_POSE_Y = 4; 

	//Speed
	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.6);
	public static final double SLOW_SPEED = 0.33;
	public static final double NORMAL_SPEED = 1;
}
