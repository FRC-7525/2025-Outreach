package frc.robot.Subsystems.Drive;

public interface DriveIO {
	public static class DriveIOInputs {

		public double leftPositionRad = 0.0;
		public double leftVelocityRadPerSec = 0.0;
		public double leftAppliedVolts = 0.0;
		public double[] leftCurrentAmps = new double[] {};

		public double rightPositionRad = 0.0;
		public double rightVelocityRadPerSec = 0.0;
		public double rightAppliedVolts = 0.0;
		public double[] rightCurrentAmps = new double[] {};
	}

	public default void updateInputs(DriveIOInputs inputs) {}

	public default void setVoltage(double leftVolts, double rightVolts) {}

	public default void setVelocity(
		double leftRadPerSec,
		double rightRadPerSec,
		double leftFFVolts,
		double rightFFVolts
	) {}
}
