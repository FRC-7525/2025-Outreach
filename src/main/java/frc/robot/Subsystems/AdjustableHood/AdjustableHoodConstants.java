package frc.robot.Subsystems.AdjustableHood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static frc.robot.GlobalConstants.ROBOT_MODE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import java.util.function.Supplier;

public class AdjustableHoodConstants {

	public static final String SUBSYSTEM_NAME = "Adjustable Hood";

	public static final Angle IDLE_ANGLE = Degrees.of(0);
	public static final Angle FIXED_ANGLE = Degrees.of(45);

	public static final Angle MAX_ANGLE = Degrees.of(60);
	public static final Angle MIN_ANGLE = Degrees.of(0);

	public static final Supplier<PIDController> PIVOT_PID = () ->
		switch (ROBOT_MODE) {
			case SIM -> new PIDController(0.1, 0, 0.01);
			default -> new PIDController(0.1, 0, 0.01);
		};

	public static final class Real {

		public static final Angle PIVOT_TOLERANCE = Degrees.of(0.5); // Random tolerance of 0.5 degrees
		public static final int PIVOT_CAN_ID = 1; // Random value
		public static final double GEAR_RATIO = 25; // Random ahh gear ratio of 25:1
	}

	public static final class Sim {

		public static final Angle PIVOT_TOLERANCE = Degrees.of(0.5); // Random tolerance of 0.5 degrees
		public static final double GEAR_RATIO = 25; // Random ahh gear ratio of 25:1
		public static final MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(1); // Random moment of inertia
		public static final Distance PIVOT_LENGTH = Inches.of(8); // Random length of 8 inches
		public static final int NUM_PIVOT_MOTORS = 1; // Number of motors used for the pivot
		public static final Angle PIVOT_STARTING_ANGLE = Degrees.of(0); // Starting angle of the pivot motor
	}
}
