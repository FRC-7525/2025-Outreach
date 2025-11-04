package frc.robot.Subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;


public interface ShooterIO {
	
	public static class ShooterIOInputs {

		public AngularVelocity shooterSetpoint;
		public double shooterVelocity;
	}

	public abstract void updateInputs(ShooterIOInputs inputs);

	public abstract void setShooterSetpoint(AngularVelocity setpoint);

	public abstract boolean atTargetSpeed();
}
