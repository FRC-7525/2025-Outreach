package frc.robot.Subsystems.Vision;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservationType;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;
import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {

	// Akit my savior
	private final VisionIO[] io;
	private final VisionIOInputs[] inputs;
	private final Alert[] disconnectedAlerts;

	List<Pose3d> allTagPoses = new LinkedList<>();
	List<Pose3d> allRobotPoses = new LinkedList<>();
	List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
	List<Pose3d> allRobotPosesRejected = new LinkedList<>();

	private static Vision instance;

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision(
				switch (ROBOT_MODE) {
					case REAL -> new VisionIO[] {
						//Replace w/ actual stuff once camera num and pos is figured out
					};
					case SIM -> new VisionIO[] {};
					case TESTING -> new VisionIO[] {
						//Replace w/ actual stuff once camera num and pos is figured out
					};
				}
			);
		}
		return instance;
	}

	private Vision(VisionIO... io) {
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputs[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputs();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert(
				"Vision camera " + Integer.toString(i) + " is disconnected.",
				AlertType.kWarning
			);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
		}

		// Initialize logging values
		allTagPoses = new LinkedList<>();
		allRobotPoses = new LinkedList<>();
		allRobotPosesAccepted = new LinkedList<>();
		allRobotPosesRejected = new LinkedList<>();

		processVision();
		// Log summary data
		// 	Logger.recordOutput(
		// 		"Vision/Summary/TagPoses",
		// 		allTagPoses.toArray(new Pose3d[allTagPoses.size()])
		// 	);
		// 	Logger.recordOutput(
		// 		"Vision/Summary/RobotPoses",
		// 		allRobotPoses.toArray(new Pose3d[allRobotPoses.size()])
		// 	);
		// 	Logger.recordOutput(
		// 		"Vision/Summary/RobotPosesAccepted",
		// 		allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()])
		// 	);
		// 	Logger.recordOutput(
		// 		"Vision/Summary/RobotPosesRejected",
		// 		allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()])
		// 	);
	}

	private void processVision() {
		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// 		// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				// 			Logger.recordOutput(
				// 				"Vision/Camera" + Integer.toString(cameraIndex) + "/Tag Count",
				// 				observation.tagCount() == 0
				// 			);
				// 			Logger.recordOutput(
				// 				"Vision/Camera" + Integer.toString(cameraIndex) + "/Ambiguous",
				// 				(observation.tagCount() == ONE_TAG && observation.ambiguity() > maxAmbiguity)
				// 			);
				// 			Logger.recordOutput(
				// 				"Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field X",
				// 				observation.pose().getX() < 0.0 ||
				// 				observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
				// 			);
				// 			Logger.recordOutput(
				// 				"Vision/Camera" + Integer.toString(cameraIndex) + "/Outside of Field Y",
				// 				observation.pose().getY() < 0.0 ||
				// 				observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
				// 			);

				// Add pose to log

				// Skip if rejected

				//Calculate standard deviations
				double stdDevFactor =
					Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				@SuppressWarnings("unused")
				double linearStdDev = linearStdDevBaseline * stdDevFactor;
				@SuppressWarnings("unused")
				double angularStdDev = angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}
				//254 standard dev

				// Send vision observation

			}

			// 		// Log camera datadata
			// 		Logger.recordOutput(
			// 			"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
			// 			tagPoses.toArray(new Pose3d[tagPoses.size()])
			// 		);
			// 		Logger.recordOutput(
			// 			"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
			// 			robotPoses.toArray(new Pose3d[robotPoses.size()])
			// 		);
			// 		Logger.recordOutput(
			// 			"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
			// 			robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()])
			// 		);
			// 		Logger.recordOutput(
			// 			"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
			// 			robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()])
			// 		);

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}
	}

	public Rotation2d getYaw() {
		return inputs[0].latestTargetObservation.tx();
	}

	public Rotation2d getPitch() {
		return inputs[0].latestTargetObservation.ty();
	}
}
