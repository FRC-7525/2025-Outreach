package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ShooterIOReal implements ShooterIO {

    private TalonFX flywheelMotor;
    private AngularVelocity shooterSetpoint;
    private PIDController flywheelController;

    public ShooterIOReal() {
        flywheelMotor = new TalonFX(FLYWHEEL_CAN_ID);
        flywheelMotor.setPosition(0); // Zero the motor position
        flywheelController = FLYWHEEL_PID.get();
        flywheelController.setTolerance(FLYWHEEL_TOLERANCE.in(RotationsPerSecond));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterSetpoint = shooterSetpoint;
        inputs.shooterVelocity = flywheelMotor.getVelocity().getValueAsDouble();

        if (ROBOT_MODE == RobotMode.TESTING) {
            SmartDashboard.putData("Flywheel PID Controller", flywheelController);
        }
    }

    @Override
    public void setShooterSetpoint(AngularVelocity shooterSetpoint) {
        this.shooterSetpoint = shooterSetpoint;
        flywheelMotor.set(
            flywheelController.calculate(
                flywheelMotor.getVelocity().getValueAsDouble(),
                shooterSetpoint.in(RotationsPerSecond)
            )
        );
    }

    @Override
    public boolean atTargetSpeed() {
        return flywheelController.atSetpoint();
    }
}
