package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.List;

import edu.wpi.first.units.measure.AngularVelocity;

public class HoodedShooterSupersystemConstants {

	public static final String SUBSYSTEM_NAME = "HoodedShooterSupersystem";

	public static final int FAKE_VALUE = 40;
	public static final int FAKE_VALUE_2 = 1;

	public static final AngularVelocity MEDIUM_SPEED = RotationsPerSecond.of(1500);

	// Random values and stuff
	public static final List<ShooterDataPoint> SHOOTER_DATA_POINTS = List.of(
		new ShooterDataPoint(RotationsPerSecond.of(1500), Degrees.of(30), Meters.of(2.0)),
		new ShooterDataPoint(RotationsPerSecond.of(2000), Degrees.of(35), Meters.of(3.0)),
		new ShooterDataPoint(RotationsPerSecond.of(2500), Degrees.of(40), Meters.of(3.5)),
		new ShooterDataPoint(RotationsPerSecond.of(3000), Degrees.of(45), Meters.of(4.0))
	);
}
