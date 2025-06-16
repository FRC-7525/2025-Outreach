package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import java.util.function.Supplier;

public class ShooterConstants {

	public static final String SUBSYSTEM_NAME = "Shooter";
	public static final double ZERO = 0.0; // Placeholder for zero value

	public static final AngularVelocity FIXED_SPEED = RotationsPerSecond.of(300);

	public static final int FLYWHEEL_CAN_ID = 2; // Example CAN ID
	// Add a feedforward controller for the flywheel
	public static final double FLYWHEEL_KS = 0.2; // Static gain (example)
	public static final double FLYWHEEL_KV = 0.12; // Velocity gain (example)
	public static final double FLYWHEEL_KA = 0.01; // Acceleration gain (example)

	public static final Supplier<SimpleMotorFeedforward> FLYWHEEL_FF = () ->
		new SimpleMotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);

	public static final AngularVelocity FLYWHEEL_TOLERANCE = RotationsPerSecond.of(50); // Random
	public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.1); // Random value
	public static final double FLYWHEEL_GEARING = 10.0; // Random gear ratio
}
