package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ShooterIOReal implements ShooterIO {

	private TalonFX flywheelMotor;
	private AngularVelocity shooterSetpoint;
	private final SimpleMotorFeedforward feedforward;

	public ShooterIOReal() {
		flywheelMotor = new TalonFX(FLYWHEEL_CAN_ID);
		flywheelMotor.setPosition(0); // Zero the motor position
		feedforward = FLYWHEEL_FF.get();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.shooterSetpoint = shooterSetpoint;
		// Get velocity in rotations per second
		inputs.shooterVelocity = flywheelMotor.getVelocity().getValueAsDouble();

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Flywheel PID Controller", null); // Removed PID controller
		}
	}

	@Override
	public void setShooterSetpoint(AngularVelocity shooterSetpoint) {
		this.shooterSetpoint = shooterSetpoint;
		double setpointRPS = shooterSetpoint.in(RotationsPerSecond);
		double ffVolts = feedforward.calculate(setpointRPS);
		flywheelMotor.setVoltage(ffVolts);
	}

	@Override
	public boolean atTargetSpeed() {
		double error = Math.abs(
			flywheelMotor.getVelocity().getValueAsDouble() - shooterSetpoint.in(RotationsPerSecond)
		);
		return error < FLYWHEEL_TOLERANCE.in(RotationsPerSecond);
	}
}
