package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
	public static class GyroIOInputs {

		public boolean connected = false;
		public Rotation2d yawPosition = Rotation2d.kZero;
		public double yawVelocityRadPerSec = 0.0;
	}

	public default void updateInputs(GyroIOInputs inputs) {}
}
