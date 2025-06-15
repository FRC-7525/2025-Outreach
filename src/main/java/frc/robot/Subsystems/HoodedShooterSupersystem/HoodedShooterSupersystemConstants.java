package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class HoodedShooterSupersystemConstants {

	public static final String SUBSYSTEM_NAME = "HoodedShooterSupersystem";

	public static final int FAKE_VALUE = 40;
	public static final int FAKE_VALUE_2 = 1;

	public static final double dt = 0.02; // increase to add more intermediate values in trajectory sim
	
	public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.812);
	public static final Distance GROUND_TO_SHOOTER = Meters.of(0.3);
    public static final Distance BALL_RADIUS = Meters.of(0.09);  // meters
    public static final Mass BALL_MASS = Kilogram.of(0.24);    // kg
    public static final MomentOfInertia BALL_INERTIA = KilogramSquareMeters.of((2.0 / 5.0) * BALL_MASS.in(Kilogram) * BALL_RADIUS.in(Meters) * BALL_RADIUS.in(Meter));
    public static final double SPIN_DECAY_CONSTANT = 0.15; // Decay of spin after release
    public static final double AIR_DENSITY = 1.225; // kg/m^3
    public static final double DRAG_COEFFICIENT = 0.47; // aprox for a sphere
    public static final double MAGNUS_COEFFICIENT = 0.2; // have to get empirically (random)
}
