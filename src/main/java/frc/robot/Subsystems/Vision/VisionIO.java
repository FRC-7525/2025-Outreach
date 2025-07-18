package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
	@AutoLog
	public static class VisionIOInputs {

		public boolean connected = false;
		public TargetObservation latestTargetObservation = new TargetObservation(
			new Rotation2d(),
			new Rotation2d()
		);
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIds = new int[0];
		public Pose2d targetPose = new Pose2d();
	}

	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(
		double timestamp,
		Pose3d pose,
		double ambiguity,
		int tagCount,
		double averageTagDistance,
		PoseObservationType type,
		double avgTagArea,
		Set<Short> tagsObserved
	) {}

	public static enum PoseObservationType {
		MEGATAG_1,
		MEGATAG_2,
		PHOTONVISION,
	}

	public default void updateInputs(VisionIOInputs inputs) {}
}
