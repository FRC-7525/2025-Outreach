package frc.robot.Subsystems.HoodedShooterSupersystem;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class HoodedShooterSupersystemConstants {

	public static final String SUBSYSTEM_NAME = "HoodedShooterSupersystem";

	public static final int FAKE_VALUE = 40;
	public static final int FAKE_VALUE_2 = 1;

	public static final AngularVelocity MEDIUM_SPEED = RotationsPerSecond.of(1500);

	// Random values and stuff
	public static final double[] ANGLES = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
	public static final double[] DISTANCES = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
}
