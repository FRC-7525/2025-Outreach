package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.ROBOT_MODE;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
    public static final AngularVelocity FIXED_SPEED = RotationsPerSecond.of(300);

    public static final int FLYWHEEL_CAN_ID = 2; // Example CAN ID

    public static final Supplier<PIDController> FLYWHEEL_PID = () -> 
        switch (ROBOT_MODE) {
            case SIM -> new PIDController(0.1, 0, 0.01);
            default -> new PIDController(0.1, 0, 0.01);
        };

    public static final AngularVelocity FLYWHEEL_TOLERANCE = RotationsPerSecond.of(50); // Random
}
