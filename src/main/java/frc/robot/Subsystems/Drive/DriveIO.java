package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;
import swervelib.SwerveDrive;

public interface DriveIO {
	@AutoLog
	public class DriveIOInputs {

		double timestamp = 0;
	}

	public default void updateInputs(DriveIOInputs inputs) {}

	public default SwerveDrive getDrive() {
		return null;
	}

	public default ChassisSpeeds getSwerveInputs() {
		return null;
	}

	public default void setSpeed() {}

	public default void zeroGyro() {}

	public default void addVisionMeasurement(
		Pose2d visionPose,
		double timestamp,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {}
}
