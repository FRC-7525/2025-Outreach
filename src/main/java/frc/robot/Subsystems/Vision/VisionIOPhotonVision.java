package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.Drive.Drive;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {

	protected final PhotonCamera camera;
	protected final Transform3d robotToCamera;

	/**
	 * Creates a new VisionIOPhotonVision.
	 *
	 * @param name The configured name of the camera.
	 * @param rotationSupplier The 3D position of the camera relative to the robot.
	 */
	public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
		camera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.connected = camera.isConnected();

		// Read new camera observations
		Set<Short> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		for (var result : camera.getAllUnreadResults()) {
			// Update latest target observation
			if (result.hasTargets()) {
				inputs.latestTargetObservation = new TargetObservation(
					Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
					Rotation2d.fromDegrees(result.getBestTarget().getPitch())
				);
			} else {
				inputs.latestTargetObservation = new TargetObservation(
					new Rotation2d(),
					new Rotation2d()
				);
			}

			// Add pose observation
			if (result.multitagResult.isPresent()) { // Multitag result
				var multitagResult = result.multitagResult.get();

				// Calculate robot pose
				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(
					fieldToRobot.getTranslation(),
					fieldToRobot.getRotation()
				);

				// Calculate average tag distance and avg tag area
				double totalTagDistance = 0.0;
				double totalTagArea = 0.0;
				for (var target : result.targets) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
					totalTagArea += target.area;
				}

				// Add tag IDs
				tagIds.addAll(multitagResult.fiducialIDsUsed);

				// Add observation
				poseObservations.add(
					new PoseObservation(
						result.getTimestampSeconds(), // Timestamp
						robotPose, // 3D pose estimate
						multitagResult.estimatedPose.ambiguity, // Ambiguity
						multitagResult.fiducialIDsUsed.size(), // Tag count
						totalTagDistance / result.targets.size(), // Average tag distance
						PoseObservationType.PHOTONVISION,
						totalTagArea / result.targets.size(),
						tagIds
					)
				); // Observation type
			} else if (!result.targets.isEmpty()) { // Single tag result
				var target = result.targets.get(0);

				// Calculate robot pose
				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
				if (tagPose.isPresent()) {
					Transform3d fieldToTarget = new Transform3d(
						tagPose.get().getTranslation(),
						tagPose.get().getRotation()
					);
					Transform3d cameraToTarget = target.bestCameraToTarget;
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
					Pose3d robotPose = new Pose3d(
						fieldToRobot.getTranslation(),
						fieldToRobot.getRotation()
					);

					// Add tag ID
					tagIds.add((short) target.fiducialId);

					// Add observation
					poseObservations.add(
						new PoseObservation(
							result.getTimestampSeconds(), // Timestamp
							robotPose, // 3D pose estimate
							target.poseAmbiguity, // Ambiguity
							ONE_TAG, // Tag count
							cameraToTarget.getTranslation().getNorm(), // Average tag distance
							PoseObservationType.PHOTONVISION,
							target.area,
							Set.of((short) target.fiducialId)
						)
					); // Observation type
				}
			}

			//Always find pose of first tag seen, regardless of multitag or not
			if (!result.targets.isEmpty()) {
				//First tag seen
				var target = result.targets.get(0);

				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
				if (tagPose.isPresent()) {
					//Offset from tag pose to get pose of bucket center
					inputs.targetPose = new Pose2d(
						tagPose.get().getX() -
						bucketOffset.in(Meters) *
						Math.cos(tagPose.get().toPose2d().getRotation().getRadians()),
						tagPose.get().getY() -
						bucketOffset.in(Meters) *
						Math.sin(tagPose.get().toPose2d().getRotation().getRadians()),
						new Rotation2d()
					);
				}
			} else {
				inputs.targetPose = new Pose2d();
			}
		}

		// Save pose observations to inputs object
		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.poseObservations[i] = poseObservations.get(i);
		}

		// Save tag IDs to inputs objects
		inputs.tagIds = new int[tagIds.size()];
		int i = 0;
		for (int id : tagIds) {
			inputs.tagIds[i++] = id;
		}
	}
}
